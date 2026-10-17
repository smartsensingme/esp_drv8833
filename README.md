# esp_drv8833

Componente ESP-IDF para uma ponte DRV8833 com dois motores DC. API independente
da placa, compatível com ESP32-C3 e ESP32-S3, implementada usando APIs públicas
da ESP-IDF 6.0. O diretório raiz já tem a estrutura de componente reutilizável;
não é necessário extrair código de `main/` para usá-lo em outro projeto.

O componente recebe duty e sentido, mantém o estado aplicado e entrega avisos
ULT. Não mede corrente/RPM, não faz controle fechado e não deduz a causa de
um sinal ULT. Não há protocolo de comunicação ou formato persistente próprio:
as structs são valores C em memória, sem ABI binária ou serialização garantida.

## Arquitetura e ciclo de vida

| Arquivo | Responsabilidade |
| --- | --- |
| `include/esp_drv8833.h` | API pública, configuração, estados e eventos com unidades/contratos. |
| `src/esp_drv8833.c` | Mapeamento lógico, serialização de comandos, sleep, reversão e captura/entrega ULT. |
| `src/pwm_backend.h` | Contrato privado comum; não deve ser chamado pela aplicação. |
| `src/pwm_ledc.c` | PWM de quatro entradas por LEDC, sem serviço de fade. |
| `src/pwm_mcpwm.c` | PWM de quatro entradas por MCPWM, com comparadores atualizados no início do período. |
| `Kconfig` e `CMakeLists.txt` | Selecionam exatamente um backend e declaram dependências de compilação/link. |
| `idf_component.yml` | Metadados e faixa de ESP-IDF suportada: `>=6.0,<6.1`, alvos C3/S3. |

O caminho de comandos é `aplicação → API → mutex → backend → periférico`.
O caminho de eventos é `ULT → ISR curta → snapshot interno → notificação →
tarefa → callback`, com callback fora dos locks. Só o caminho de comandos
altera PWM/EEP; a ISR e o worker ULT não param os motores.

1. Prepare a configuração e qualquer contexto de callback antes de criar a
   instância. Reserve os pinos e recursos de PWM exclusivamente.
2. `esp_drv8833_create()` copia a configuração, aloca a instância em RAM
   interna, cria um mutex e um semáforo de término, configura saídas baixas e
   inicializa o backend. Com ULT, cria também uma tarefa de 3072 bytes de stack
   solicitados, prioridade 5/Core 0, antes de habilitar a interrupção.
3. O estado inicial é coast e, com EEP conectado, sleep. Um comando não nulo
   desperta a ponte e aplica os duties. Comandos zero por roda e reversões
   seguem as regras da seção “PWM e controle”.
4. A sinalização ULT tem estado independente: armada → evento capturado/latch
   marcado → evento copiado pelo worker → reconhecimento pela aplicação →
   armada. Entrega não limpa o latch; reconhecimento não muda motores.
5. Antes de destruir, impeça chamadas concorrentes. A destruição marca a
   instância como closing, tenta parar, remove o handler, sincroniza o término
   do worker e libera backend/objetos. EEP inicializado permanece como saída
   baixa. O serviço global GPIO fica instalado, mesmo se criado pelo driver.

O callback inicial pode começar assim que a instância é publicada, antes de
`create()` retornar ao chamador; não inicialize seu contexto somente depois
dessa chamada. O objeto de configuração pode sair de escopo após a criação,
mas os dados apontados por `callback_context` continuam pertencendo à aplicação.

Se a destruição falhar antes da limpeza final, por timeout de worker ou erro
ao remover o handler, a instância closing fica disponível para consulta e
nova tentativa de destruição. Se chegar à limpeza final, o handle é liberado
mesmo quando o retorno indica erro de parada/backend. Nesse último caso não
reutilize o ponteiro; uma falha de cleanup pode deixar recurso IDF sem liberar.
Erro de argumento/core/callback ou timeout do mutex inicial não inicia a
destruição. Veja o contrato completo em `esp_drv8833.h`.

## API pública e erros

Todas as funções públicas são para contexto de tarefa. O callback pode usar
comandos, consulta e reconhecimento; não pode destruir sua própria instância.

| Função | Efeito | Retornos relevantes |
| --- | --- | --- |
| `esp_drv8833_create(&config, &driver)` | Cria uma instância; `driver` fica `NULL` em erro. | Argumento/configuração inválida, falta de memória/tarefa, recurso ocupado ou erro de GPIO/PWM. |
| `esp_drv8833_set_speed(driver, right, left)` | Aplica duties lógicos independentes; rejeita valores fora da faixa em vez de limitar silenciosamente. | `ESP_ERR_INVALID_ARG`, timeout de mutex, closing ou primeiro erro de escrita; nesse último caso tenta parar ambos. |
| `esp_drv8833_coast(driver)` | Equivale ao comando `(0,0)`, incluindo sleep opcional. | Mesmos erros de comando; não executa frenagem ativa. |
| `esp_drv8833_get_state(driver, &state)` | Copia estado lógico, latch, contador e último evento; lê o nível ULT separadamente. | Argumento inválido ou timeout de mutex; destino não é alterado nesses erros de entrada. |
| `esp_drv8833_clear_fault(driver)` | Limpa somente o latch, sem PWM/EEP/reset de contador. | Argumento inválido, timeout ou `ESP_ERR_INVALID_STATE` se evento ainda aguarda cópia ou a instância está closing. |
| `esp_drv8833_delete(driver)` | Tenta parar e encerra a propriedade dos recursos conforme a fase alcançada. | Argumento/core/callback inválido, timeout, remoção de handler ou primeiro erro de parada/cleanup; observe a regra de validade do handle acima. |

Consulte `esp_err_to_name(err)` ao registrar erros fora do caminho crítico.
Não interprete `ESP_OK` como medição elétrica das saídas. Em um erro de escrita,
o retorno preserva o erro original e `state.last_stop_error` informa a tentativa
de parada posterior. O callback ULT não é um canal genérico de erros da API.

## PWM e controle

Em `menuconfig → Component config → DRV8833 motor driver → PWM peripheral`:

| Alvo | Padrão | Alternativa |
| --- | --- | --- |
| ESP32-C3 | LEDC | MCPWM indisponível |
| ESP32-S3 | MCPWM | LEDC |

A escolha vale para todas as instâncias do componente. Ambos geram PWM por
hardware, sem interrupção por borda nem tarefa periódica. MCPWM oferece uma
base de tempo voltada a motores; não foi medido menor custo de CPU em relação
ao LEDC. Esta versão utiliza GPIO/worker para tratar ULT nos dois backends;
não configura o mecanismo de falha/freio por hardware do MCPWM.

O padrão é 20 kHz: MCPWM usa resolução de 80 MHz, com 4000 ticks por período;
LEDC usa XTAL de 40 MHz e resolução automática (10 bits, 1024 passos a 20 kHz).
O duty é arredondado para o passo mais próximo. Um comando muito pequeno pode
virar duty zero pela quantização; o sleep automático depende de ambos os
**comandos** serem exatamente zero. MCPWM exige período inteiro de 2..65535
ticks. LEDC rejeita frequências não suportadas e aquelas cujo valor retornado
por `ledc_get_freq()` difere do solicitado; essa leitura inteira não representa
uma medição física nem garante ausência de erro sub-hertz.

Essas validações são do periférico/backend, não uma certificação de que todo
valor aceito será eletricamente adequado ao DRV8833 e ao motor. Use a frequência
validada na sua montagem; a referência de bancada deste componente é 20 kHz.

Cada motor tem duty independente; a frequência da portadora é compartilhada.
`esp_drv8833_set_speed(driver, direita, esquerda)` aceita `float` finito entre
-100 e +100. Valores inválidos são rejeitados, sem saturação silenciosa.
Comandos positivos e negativos escolhem entradas opostas da ponte. É possível
usar, por exemplo, `(30, 60)` ou `(30, -30)`. As duas atualizações não são
atômicas em hardware, mesmo quando compartilham uma base de tempo.

Em DRIVE, uma entrada recebe PWM e a outra fica baixa: modo fast decay/coast.
Zero deixa ambas baixas (roda livre). Não há frenagem ativa nesta API. Na
reversão de um comando não nulo, ambas as entradas daquele motor são forçadas
baixas antes de um intervalo configurável de coast, 50 us por padrão. Isso
não limita corrente mecanicamente nem substitui uma rampa de controle.

`left_bridge` escolhe qual ponte física representa a roda esquerda: A é
IN1/IN2 e B é IN3/IN4; a direita usa a outra. `invert_left` e `invert_right`
invertem separadamente o significado do comando em relação à ligação dos
fios. Ajuste a configuração antes de criar a instância; para remapear depois,
pare, destrua e recrie. Não se altera a convenção durante movimento.

## EEP e ULT opcionais

Use GPIO `-1` quando um sinal não estiver conectado ao ESP32.

* **EEP/nSLEEP:** baixo quando ambos os comandos são zero; alto antes de
  acionar qualquer motor. Ao despertar, o driver mantém PWM baixo durante
  1 ms. Sem EEP, zero ainda seleciona coast, mas não sleep. Nesse caso o módulo
  precisa manter nSLEEP alto por hardware para funcionar.
* **ULT/nFAULT:** entrada ativa em baixo, com pull-up interno habilitado.
  O sinal é open-drain; o pull-up deve referenciar 3,3 V. A primeira borda de
  descida marca `fault_latched` e agenda um evento na tarefa do Core 0,
  prioridade 5. A tarefa chama `on_fault`, se configurado, sem escrever PWM
  ou EEP. ULT já baixo na criação também produz uma sinalização inicial.

ULT é apenas uma sinalização para a aplicação: **não para motores, não altera
sleep e não impede comandos de velocidade**, mesmo quando permanece baixo.
O sinal não identifica separadamente a causa nem qual motor originou a
ocorrência. A aplicação define sua política, inclusive quando habilitar o
controle após alimentar a ponte. As proteções elétricas internas do DRV8833
continuam independentes dessa política de software.

Enquanto `fault_latched` estiver marcado, novas bordas não agendam eventos
nem repetem o callback. O contador `fault_falling_edges` continua contando
bordas atendidas, com wrap em 32 bits. A aplicação chama
`esp_drv8833_clear_fault()` para reconhecer a sinalização e permitir outra.
Não há reconhecimento automático, debounce ou garantia de que todo pulso
seja uma falha elétrica real.

O reconhecimento limpa somente o latch: não altera PWM, EEP, comandos ou o
contador. Pode ser feito mesmo com ULT baixo; nesse caso outra notificação
exige que o sinal volte ao alto e depois apresente uma nova borda de descida.
Não há re-notificação imediata do nível baixo existente. O latch e
`fault_pin_low` são distintos: o primeiro guarda o histórico ainda não
reconhecido; o segundo mostra o nível elétrico atual.

Para não perder um evento aceito antes da entrega, `clear_fault()` retorna
`ESP_ERR_INVALID_STATE` enquanto `fault_event_pending` estiver verdadeiro.
O worker limpa esse indicador ao copiar o evento, antes de chamar o callback.
Assim, o reconhecimento pode ser feito dentro de `on_fault()` ou por uma
consulta posterior ao estado. Com callback `NULL`, o worker também consome o
evento e a aplicação pode usar somente polling de estado. O evento já copiado
pode estar em execução no callback quando outra tarefa reconhece o latch.

`on_fault` executa fora do mutex e nunca em ISR. Mantenha-o curto; pode consultar
estado, chamar `esp_drv8833_coast()` se desejar parar e reconhecer a sinalização,
mas não destruir o driver. Seu contexto deve existir até a destruição terminar.
O exemplo registra o contexto de cada sinal e reconhece explicitamente no
callback, conforme o teste de bancada; não para os motores. Para testar com
ULT, conecte-o ao GPIO configurado
(padrão GPIO7); `fault_gpio = -1` continua disponível para uso sem esse sinal.

Exemplo de tratamento explícito pela aplicação, após consultar o estado:

```c
esp_drv8833_state_t state;
ESP_ERROR_CHECK(esp_drv8833_get_state(driver, &state));
if (state.fault_latched && !state.fault_event_pending) {
    // Política opcional da aplicação: parar antes de reconhecer.
    ESP_ERROR_CHECK(esp_drv8833_coast(driver));
    ESP_ERROR_CHECK(esp_drv8833_clear_fault(driver));
}
```

Erros de escrita de PWM/GPIO continuam retornando erro ao chamador e provocam
uma tentativa de parada de ambos. Eles não marcam o latch ULT nem geram seu
callback; a aplicação pode tratar o erro e tentar novo comando. Consulte
`last_stop_error` para a tentativa de parada mais recente: um erro de hardware
não permite concluir que as saídas físicas pararam.

A sinalização depende de interrupções/escalonamento e não tem prazo rígido de
entrega. Se o serviço GPIO não for IRAM, operações de flash podem adiar a ISR.
Mesmo com ISR em IRAM, o callback em tarefa não tem garantia de executar
durante essas operações. O evento é copiado antes de liberar o mutex; novas
ocorrências reconhecidas durante um callback aguardam seu término para entrega.

### Contexto temporal do evento ULT (versão 0.3.0)

Todos os eventos continuam sendo sinalizados, inclusive o primeiro próximo de
uma partida ou coast. A aplicação recebe contexto para decidir sua política;
o driver não presume que um pulso durante uma transição seja inofensivo.

| `event->context` | Associação temporal |
| --- | --- |
| `ESP_DRV8833_FAULT_CONTEXT_NONE` | Captura fora de uma transição e de sua janela posterior; não significa diagnóstico de causa. |
| `ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION` | Inicialização da instância em `create()`, incluindo ULT já baixo ao habilitar a entrada. |
| `ESP_DRV8833_FAULT_CONTEXT_WAKE` | Saída de sleep: EEP baixo → alto antes de aplicar o comando. |
| `ESP_DRV8833_FAULT_CONTEXT_MOTOR_START` | Uma roda passou de comando zero para não zero com a ponte já acordada, ou sem EEP configurado. |
| `ESP_DRV8833_FAULT_CONTEXT_COAST` | Uma ou ambas as rodas passaram para zero, incluindo EEP alto → baixo quando ambas param; também uma tentativa de parada por erro quando havia acionamento. |
| `ESP_DRV8833_FAULT_CONTEXT_REVERSAL` | Inversão direta do sinal de um comando não nulo, incluindo o intervalo de coast intermediário. |

`fault_context_window_us` define a janela **após terminar a operação**, 2000 us
por padrão. Eventos durante a operação também recebem seu contexto, mesmo se
a tarefa for preemptada. Com valor zero, só se associam eventos capturados
durante a operação. Essa janela é uma heurística configurável, não um limite
elétrico caracterizado para o módulo.

A associação usa a transição mais recente. Quando um comando muda várias
coisas, a prioridade é WAKE, REVERSAL, MOTOR_START e COAST. Uma simples mudança
de duty entre dois valores não nulos do mesmo sinal não abre nem prolonga
janela. Repetir coast quando já está em zero/sleep também não a prolonga.
Reconhecer ULT não muda a janela. Alimentar VCC manualmente não é uma operação
observada pelo driver: use o estado de alimentação conhecido pela aplicação
para interpretar esse caso.

O evento contém:

| Campo | Significado |
| --- | --- |
| `falling_edges` | Contador cumulativo de bordas atendidas, congelado na aceitação do evento; pode ser zero para o nível baixo inicial. |
| `context` | Uma das associações acima. |
| `timestamp_us` | Timestamp de `esp_timer_get_time()` na captura pela ISR ou na leitura inicial; não é o instante físico exato da borda nem a hora do callback. |
| `transition_elapsed_us` | Tempo desde o início da transição até a captura, em us, saturado em `UINT32_MAX`. Inclui a duração da operação e eventual janela posterior. Zero quando `context == NONE`. |
| `transition_active` | `true` se a captura ocorreu durante a operação; `false` na janela posterior ou quando não há associação. |

Os campos são capturados juntos e não mudam se outra operação ocorrer antes
do callback. Para polling, `state.last_fault_event` conserva o último evento
aceito, mesmo após reconhecimento; fica zerado antes da primeira sinalização.
`state.fault_falling_edges` continua mostrando o contador atual, que pode ser
maior que o congelado no evento. A leitura de estado copia o snapshot sob
sincronização; timestamps e metadados acrescentam custo limitado à ISR e às
transições, a medir na validação de hardware.

Exemplo de seleção de política no callback:

```c
if (event->context == ESP_DRV8833_FAULT_CONTEXT_NONE) {
    // Tratar conforme a política da aplicação; por exemplo, parar:
    ESP_ERROR_CHECK(esp_drv8833_coast(driver));
} else {
    // Registrar a associação; ela não comprova ausência de falha real.
}
ESP_ERROR_CHECK(esp_drv8833_clear_fault(driver));
```

### Mudança de comportamento na versão 0.2.0

ULT passou de parada/bloqueio automático para sinalização reconhecida pela
aplicação. `fault_latched` agora indica aviso pendente de reconhecimento;
`clear_fault()` apenas rearma a sinalização, sem testar ULT acordando a ponte.
O campo `stop_error` foi removido de `esp_drv8833_fault_event_t`, pois o evento
não executa mais uma parada. `last_stop_error` permanece no estado para
operações que realmente tentam parar. Aplicações que desejam a política antiga
devem chamar `coast()` no seu tratamento do evento e decidir quando retomar.

## Propriedades de `esp_drv8833_config_t`

A struct `esp_drv8833_config_t` configura uma instância que controla as duas
pontes físicas do DRV8833. Inicialize com
`esp_drv8833_config_t config = ESP_DRV8833_CONFIG_DEFAULT();` e ajuste os
campos antes de chamar `esp_drv8833_create(&config, &driver)`.

Os padrões abaixo são os da macro do componente, não os GPIOs escolhidos pelo
exemplo em `menuconfig`. Os quatro GPIOs INx são obrigatórios; a macro deixa-os
em `-1` para exigir que a aplicação escolha sua ligação.

| Propriedade | Tipo | Padrão | Descrição e valores aceitos |
| --- | --- | --- | --- |
| `in1_gpio` | `int` | `-1` | GPIO de saída ligado a IN1, primeira entrada da ponte A (Motor 1). Deve ser um GPIO de saída válido; `-1` não é aceito na criação. |
| `in2_gpio` | `int` | `-1` | GPIO de saída ligado a IN2, segunda entrada da ponte A (Motor 1). Deve ser um GPIO de saída válido; `-1` não é aceito na criação. |
| `in3_gpio` | `int` | `-1` | GPIO de saída ligado a IN3, primeira entrada da ponte B (Motor 2). Deve ser um GPIO de saída válido; `-1` não é aceito na criação. |
| `in4_gpio` | `int` | `-1` | GPIO de saída ligado a IN4, segunda entrada da ponte B (Motor 2). Deve ser um GPIO de saída válido; `-1` não é aceito na criação. |
| `sleep_gpio` | `int` | `-1` | GPIO de saída ligado a EEP (`nSLEEP`). `-1` indica ausência. Quando presente, fica baixo com ambos os comandos em zero e alto para acionar; sair do sleep acrescenta espera de 1 ms. Sem esse GPIO, a aplicação deve garantir a habilitação elétrica da ponte. |
| `fault_gpio` | `int` | `-1` | GPIO de entrada ligado a ULT (`nFAULT`), ativo em baixo, com pull-up interno habilitado. `-1` desabilita a monitoração de ULT, a ISR e a tarefa de falhas. Quando presente, gera uma sinalização latched, sem parar motores ou bloquear comandos. A aplicação reconhece com `esp_drv8833_clear_fault()`. |
| `left_bridge` | `esp_drv8833_bridge_t` | `ESP_DRV8833_BRIDGE_A` | Seleciona a ponte da roda esquerda: `ESP_DRV8833_BRIDGE_A` para IN1/IN2 ou `ESP_DRV8833_BRIDGE_B` para IN3/IN4. A roda direita usa automaticamente a outra ponte. É um enum, não um flag booleano. |
| `invert_left` | `bool` | `false` | Inverte o sinal do comando da roda esquerda antes de escolher a entrada PWM. `false` preserva a polaridade; `true` troca o sentido elétrico correspondente a comandos positivos/negativos. Não muda o duty em módulo. |
| `invert_right` | `bool` | `false` | Mesmo comportamento de inversão, aplicado independentemente à roda direita. |
| `pwm_freq_hz` | `uint32_t` | `20000` | Frequência da portadora em hertz, compartilhada pelos dois motores; os duties continuam independentes. Deve ser maior que zero e suportada pelo backend. MCPWM exige que `80000000 / pwm_freq_hz` seja inteiro entre 2 e 65535; LEDC escolhe a resolução automaticamente e verifica a frequência retornada pelo periférico. Veja os detalhes em “PWM e controle”. |
| `reversal_coast_us` | `uint32_t` | `50` | Intervalo em microssegundos com as duas entradas do motor baixas ao inverter diretamente um comando não nulo. Aceita `0` a `1000`; `0` elimina a espera adicional. É um intervalo de coast por software, não o dead time interno do CI nem uma rampa de velocidade. A espera ocorre na tarefa chamadora. |
| `fault_context_window_us` | `uint32_t` | `2000` | Janela em microssegundos após uma transição para associar temporalmente ULT. Aceita `0..1000000`; zero associa somente durante a operação. Não filtra sinais, não muda o latch e não decide a política da aplicação. Configurado em C, não em Kconfig. |
| `ledc_timer` | `int` | `0` | Apenas LEDC: timer reservado exclusivamente pela instância, de `0` a `3`. Ignorado em MCPWM. Não compartilhe esse timer com outros consumidores. |
| `ledc_first_channel` | `int` | `0` | Apenas LEDC: primeiro de quatro canais consecutivos usados para IN1, IN2, IN3 e IN4. Aceita `0..2` no C3 (seis canais) e `0..4` no S3 (oito canais). Todos os quatro canais precisam estar livres e reservados para a instância. Ignorado em MCPWM. |
| `mcpwm_group` | `int` | `0` | Apenas MCPWM: grupo no qual serão alocados um timer, dois operadores e quatro comparadores/geradores. No S3 aceita `0` ou `1`, sujeitos à disponibilidade e compatibilidade dos recursos. Ignorado em LEDC; C3 não oferece MCPWM. |
| `on_fault` | `esp_drv8833_fault_callback_t` | `NULL` | Callback opcional dos eventos ULT. Exige `fault_gpio >= 0`; configurá-lo com ULT ausente faz a criação retornar `ESP_ERR_INVALID_ARG`. Com ULT presente e callback `NULL`, o latch pode ser consultado pelo estado. O callback ocorre uma vez por reconhecimento da aplicação. Executa na tarefa de falhas, fora do mutex, nunca em ISR. |
| `callback_context` | `void *` | `NULL` | Ponteiro repassado como terceiro argumento de `on_fault`. O driver não copia o objeto apontado nem assume sua propriedade. Inicialize esse objeto antes da criação e mantenha-o válido até terminar a destruição. Pode ser `NULL` se o callback não precisar de contexto. |

Todos os GPIOs presentes devem ser distintos e de uso exclusivo da instância.
Além da validação de GPIO do chip, a aplicação deve conferir pinos reservados
pela placa, strapping e conexões externas. Um GPIO numericamente válido não
garante que esteja disponível naquela montagem.

A seleção LEDC/MCPWM pertence ao Kconfig do componente e não é um campo dessa
struct. No exemplo, somente os pinos são configurados pelo Kconfig do
aplicativo; mapeamento e inversões são definidos em C.

`esp_drv8833_create()` copia a configuração. A struct original pode ser local
e sair de escopo após a chamada; alterá-la depois não reconfigura a instância.
Essa cópia é rasa para `callback_context`, cujo objeto permanece pertencendo
à aplicação. Para alterar mapeamento, polaridades, GPIOs ou parâmetros de PWM,
pare os motores, destrua a instância e crie outra com a nova configuração.

A assinatura do callback é:

```c
void on_fault(esp_drv8833_handle_t driver,
              const esp_drv8833_fault_event_t *event,
              void *context);
```

`driver` identifica a instância. `event->falling_edges` informa o contador
cumulativo de bordas ULT atendidas (com wrap em 32 bits).
O contador é capturado quando a ocorrência é aceita, não quando o callback
executa. Uma sinalização inicial por ULT já baixo pode ter contador zero.
Novas bordas enquanto o latch está marcado atualizam o contador do estado,
mas não repetem o callback nem alteram o evento capturado. O ponteiro `event` só
vale durante a chamada: copie seu conteúdo se precisar retê-lo. O callback
deve ser curto e não pode destruir a instância; consulte também “EEP e ULT
opcionais” para a política de reconhecimento explícito. A destruição pode
descartar um evento que ainda não foi despachado.

## Instalação e uso

Copie este diretório para `components/esp_drv8833` do aplicativo, ou acrescente
seu caminho a `EXTRA_COMPONENT_DIRS` antes de incluir `project.cmake`. O exemplo
usa a segunda forma. O `CMakeLists.txt` do consumidor declara
`REQUIRES esp_drv8833`. O `idf_component.yml` na raiz declara a compatibilidade
com ESP-IDF, sem dependências externas. Quando houver uma revisão publicada,
ela poderá ser referenciada pelo `main/idf_component.yml` do consumidor; fixe
um commit/tag testado e já publicado. O repositório do componente é
[smartsensingme/esp_drv8833](https://github.com/smartsensingme/esp_drv8833).
Use a URL pública `https://github.com/smartsensingme/esp_drv8833.git` nas
declarações de aquisição; aliases SSH e chaves pertencem à configuração local
de autenticação do desenvolvedor, não aos arquivos do componente.

Na instalação manual com o Component Manager desativado, configure/compile com
`IDF_COMPONENT_MANAGER=0 idf.py build`. Isso não obtém dependências de rede;
os diretórios locais precisam estar disponíveis antes da configuração. O
componente não depende de outros componentes externos, só de componentes IDF.
O `main/idf_component.yml` pertence ao consumidor quando ele escolher aquisição
pelo Component Manager; o manifest desta raiz pertence ao componente reutilizável.

Exemplo de configuração da placa descrita em `docs/pins.txt`:

```c
#include "esp_drv8833.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    esp_drv8833_config_t c = ESP_DRV8833_CONFIG_DEFAULT();
    c.in1_gpio = 0;
    c.in2_gpio = 1;
    c.in3_gpio = 2;
    c.in4_gpio = 3;
    c.sleep_gpio = 10;
    c.fault_gpio = 7;
    c.left_bridge = ESP_DRV8833_BRIDGE_A;
    c.invert_left = false;
    c.invert_right = false;

    // Sem callback, a aplicação ainda pode consultar o latch e o último evento.
    esp_drv8833_handle_t driver = NULL;
    ESP_ERROR_CHECK(esp_drv8833_create(&c, &driver));

    ESP_ERROR_CHECK(esp_drv8833_set_speed(driver, 30.0f, 30.0f));
    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_drv8833_state_t state;
    ESP_ERROR_CHECK(esp_drv8833_get_state(driver, &state));
    // Interpretar state.fault_latched/last_fault_event conforme a aplicação.
    // Para rearmar sem destruir, aguardar !state.fault_event_pending e chamar
    // esp_drv8833_clear_fault(driver). Nenhum clear é necessário para encerrar.

    ESP_ERROR_CHECK(esp_drv8833_coast(driver));
    ESP_ERROR_CHECK(esp_drv8833_delete(driver));
    driver = NULL; // Não reutilizar o handle após a limpeza final.
}
```

Todos os GPIOs devem ser distintos, válidos e exclusivos. LEDC usa um timer e
quatro canais consecutivos, especificados por `ledc_timer` e
`ledc_first_channel`; não reutilize esses recursos em outro componente. Em
C3/S3 o clock LEDC é compartilhado entre timers, portanto outros consumidores
LEDC devem usar uma fonte compatível com XTAL. MCPWM aloca dinamicamente um
timer, dois operadores e quatro comparadores/geradores no `mcpwm_group`
escolhido; a falta de recursos é reportada na criação. Instâncias adicionais
precisam de GPIOs/recursos distintos; no C3 os seis canais LEDC não comportam
duas instâncias completas de quatro canais.

Criação/destruição alocam/liberam recursos. Comandos não alocam nem registram
logs. APIs são chamadas em tarefa, serializadas por mutex com espera de até
20 ms solicitados e arredondados para ticks, com retorno `ESP_ERR_TIMEOUT`
em contenção; preempção/escalonamento acrescentam latência e não há prazo
máximo de retorno garantido. Wake
custa 1 ms de busy wait; reversão custa até 1 ms, conforme configuração.
Esses custos devem entrar no orçamento de qualquer futuro loop de controle.
Não há caminho de comando certificado para cache desligado. A criação
inicializa todas as saídas em coast; a destruição preserva EEP como saída baixa.
O serviço global de ISR GPIO permanece instalado para não afetar outros donos.
Exclua chamadas concorrentes durante destruição. Um callback que não termina
pode causar timeout na destruição; nesse caso a instância permanece válida
apenas para consulta e nova tentativa de destruição, com comandos inibidos.

Com ULT habilitado, execute criação e destruição em tarefa fixada no mesmo
core do serviço ISR GPIO. O exemplo faz isso no Core 0. Se a aplicação já
instalou esse serviço, deve garantir que ele pertence ao mesmo core antes de
criar o driver. A destruição rejeita um core diferente do usado na criação;
isso evita liberar contexto enquanto uma ISR do outro core ainda o utiliza.
Os comandos e consultas podem ser feitos por tarefas em qualquer core.

### Estado retornado e propriedade dos snapshots

| Campo de `esp_drv8833_state_t` | Interpretação |
| --- | --- |
| `right_percent` / `left_percent` | Comandos lógicos percentuais da última aplicação completa, antes da inversão; não são RPM nem medição de tensão. Inicialmente zero, e zerados após parada bem-sucedida. |
| `sleeping` | Cache do último nível EEP aplicado com sucesso; sempre falso sem EEP. Não mede se o CI está fisicamente acordado. |
| `fault_latched` | Sinal aceito ainda não reconhecido; nunca inibe acionamento. |
| `fault_event_pending` | Worker ainda não copiou o evento; reconhecimento nesse estado é recusado. |
| `fault_pin_low` | Leitura elétrica instantânea de ULT, independente do latch; falso sem pino. |
| `fault_falling_edges` | Contador atual de bordas atendidas, com wrap em 32 bits; inclui repetições enquanto o latch está marcado. |
| `last_fault_event` | Cópia do último evento aceito; permanece após reconhecimento, inicialmente zerada. Os campos são descritos na seção de contexto temporal. |
| `last_stop_error` | Resultado da tentativa de parada mais recente, `ESP_OK` inicialmente; não é um diagnóstico do circuito. |

A aplicação fornece a memória de destino. A struct retornada e seu evento
embutido não guardam ponteiros para buffers internos, podendo ser copiados ou
retidos por valor. O nível ULT é lido separadamente dos campos de software;
essa consulta não é uma captura física simultânea das saídas e da entrada.

### Integração em controle periódico e energia

O componente não cria timer de controle nem possui uma taxa de comandos
obrigatória. O aplicativo deve definir sua taxa, deadline, atraso aceitável e
política de amostras/avisos antes de integrar uma malha. A portadora de PWM não
é a taxa da malha. Mesmo em 20 kHz, não há ISR periódica por pulso neste driver.

Inicie recursos fora da fase periódica e considere o custo de mutex, escritas
sequenciais, captura/snapshot e transições. Wake solicita 1 ms de busy wait e
reversão até 1 ms adicional conforme a configuração; nem esses valores nem o
mutex estabelecem WCET sob preempção. Evite logging no dono crítico; o callback
é executado no worker, mas seu custo e prioridades ainda precisam ser medidos.
O mutex é obrigatório nesta implementação; não existe opção para removê-lo.

A instância e os metadados usados pela ISR são alocados em RAM interna; o
handler e o helper de captura têm `IRAM_ATTR`. Isso não certifica o caminho
inteiro da aplicação para cache desligado. O serviço GPIO instalado sem flags
IRAM pode adiar a entrega durante flash, e o scheduler pode suspender tarefas
mesmo com ISR em IRAM. Não há ponto flutuante no handler. Mantenha watchdogs e
valide a configuração efetiva de clocks, afinidade e concorrência do app.

Sleep neste README é **sleep do DRV8833 por EEP**, não light sleep/deep sleep
do ESP32. O driver não cria locks próprios de gerenciamento de energia; usa
os mecanismos dos periféricos IDF. LEDC usa XTAL e o modo de sleep padrão do
canal; não foi configurado para manter PWM durante light sleep do ESP32.
Valide DFS/sleep e o backend escolhido antes de habilitá-los em uma malha ativa.

## Exemplo básico

```sh
cd examples/basic
idf.py set-target esp32c3    # ou esp32s3
idf.py menuconfig
idf.py build
idf.py -p SUA_PORTA flash monitor
```

Para o C3 SuperMini testado, `sdkconfig.defaults.esp32c3` seleciona flash de
4 MB e gravação pela ROM (`CONFIG_ESPTOOLPY_NO_STUB=y`), pois o esptool 5.4.0
travou ao iniciar o stub nesta montagem. Isso também permite usar normalmente
`idf.py flash`; não é uma limitação geral de todas as placas C3. Defaults não
substituem escolhas de um `sdkconfig` existente: nesse caso ajuste em
`Serial flasher config` as opções `Disable download stub` e flash de 4 MB.

Em `DRV8833 basic example`, configure apenas os GPIOs. O mapeamento e as
inversões são definidos em C na criação: Motor 1 à esquerda, com
`invert_left = false` e `invert_right = false`. A seleção LEDC/MCPWM permanece
no menu do componente, conforme o alvo.
Os padrões são os do C3 SuperMini em `docs/pins.txt`; ao usar outra placa,
confira as ligações. No C3, GPIO2/8/9 têm funções de strapping. O exemplo aguarda
três segundos e aplica 70% em `main.c`, aguarda nove segundos, executa coast,
aguarda dois segundos e aciona novamente a 75%, conforme o teste de bancada.
Faça a primeira
verificação com as rodas suspensas. Sem pino ULT, não há detecção de falha
pelo ESP32. Verifique também o jumper J1 do módulo: nSLEEP não pode estar
forçado à alimentação enquanto um GPIO tenta colocá-lo em baixo.

O percentual é duty, não velocidade. Para igualar velocidades reais, o
aplicativo maior
precisará de realimentação. O `esp-mt6701` já aceita duas estruturas
`mt6701_dev_t`, cada uma com seu handle I2C registrado em 0x06/0x46 no mesmo
barramento. Use `mt6701_init_with_timeout()` com timeout finito; `mt6701_init()`
preserva espera infinita por compatibilidade. A inversão de sentido dos encoders
também deve acompanhar a convenção física escolhida para as rodas. Este driver
de potência não depende dos sensores e não contém regulador de velocidade.

## Diagnóstico de integração

| Sintoma | Verificação |
| --- | --- |
| Criação rejeita configuração | Atribuir quatro GPIOs válidos/distintos; opcionais aceitam somente `-1` ou GPIO válido; callback não pode exigir ULT ausente. Conferir ranges específicos do backend. |
| Falta de recursos PWM | Reservar timer/canais LEDC exclusivos ou grupo MCPWM com capacidade; verificar consumidores anteriores e o clock LEDC compartilhado. |
| Comando rejeitado | Conferir NaN/infinito/faixa, timeout de mutex e fase closing; ULT baixo/marcado não é motivo para rejeição. |
| Não há callback ULT | Conferir `fault_gpio`, GND/pull-up, se o latch foi reconhecido e se houve nova borda. ULT ainda baixo após clear não repete automaticamente; ler `fault_pin_low`. |
| `clear_fault()` retorna estado inválido | Worker ainda não copiou o evento, ou shutdown começou. Consultar `fault_event_pending` ou reconhecer dentro do callback. Não há clear por ISR. |
| Contexto inesperado | Conferir instante de serviço da ISR, janela posterior e transições recentes. Mudança manual de VCC não é conhecida pelo driver; contexto não comprova causalidade. |
| Deleção retorna timeout | Callback pode estar demorado; não liberar seu contexto nem reutilizar o handle closing para comandos. Aguardar e repetir a destruição conforme o contrato. |
| Motores não giram com duty baixo | Duty não é velocidade; conferir tensão nominal, alimentação sob carga, mecânica e PWM real antes de atribuir ao driver ou elevar a tensão. |
| Avisos em partida/coast | Registrar waveform ULT/EEP/alimentação; não considerar todo pulso inofensivo por proximidade temporal. Aplicação decide a política. |

Depois de dano elétrico, confirmar reparo e desacoplamento local antes de usar
os resultados como validação do software. O capacitor da fonte, separado por
fios, não comprova desacoplamento adequado no CI. Os limites do CI não garantem
que todos os componentes de um módulo comercial tenham a mesma tensão nominal.

## Validação

Compilações verificadas com ESP-IDF 6.0: C3/LEDC, S3/MCPWM e S3/LEDC.
Os testes host em `tests/` passaram com AddressSanitizer e UBSan e
exercitam política de comandos/falhas com PWM/GPIO/RTOS simulados; não simulam
escalonamento, concorrência real, portadora ou características elétricas.
O usuário confirmou funcionamento de bancada dos comandos e do contexto ULT.
Ainda não há registro quantitativo de máximos, latências, jitter ou testes de
fault/concorrência sob carga; isso não constitui prova de pior caso.
A matriz de validação física a registrar inclui: duty/sentido nos dois canais, reversão,
wake, sinalização única de ULT persistente/pulsado, reconhecimento explícito
e latência do callback sob carga. Verifique os contextos durante e após cada
transição e fora da janela; o tempo de serviço da ISR pode afetar a associação.
ULT não deve alterar as saídas de PWM ou EEP por conta própria.
Use captura externa para medir o intervalo ULT → tratamento pela aplicação.

Referências: [DRV8833](https://www.ti.com/lit/ds/symlink/drv8833.pdf),
[MCPWM S3](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/api-reference/peripherals/mcpwm.html),
[LEDC C3](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32c3/api-reference/peripherals/ledc.html).
