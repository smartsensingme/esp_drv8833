# Histórico de alterações

## 2026-10-18 — Frenagem explícita e coast acordado (não publicado)

Alterações desde 527f5c17b118a4067e2671048d8965da86309499:

- Adiciona `esp_drv8833_brake()`: mantém as quatro entradas HIGH estático com
  a ponte acordada. LEDC usa idle HIGH; MCPWM usa force HIGH persistente.
- Adiciona `esp_drv8833_coast_awake()`: entradas LOW, com EEP HIGH quando
  configurado, distinguindo roda livre de sleep.
- Acrescenta `state.braking` e o contexto de evento ULT `BRAKE`. Um novo comando
  de movimento sai da frenagem através de coast. Erros tentam coast/sleep.
- Amplia testes nativos de transições e falhas nas escritas, e documenta a
  seleção de política ULT pelo consumidor. O runner usa binário temporário e
  seleciona o SDK do macOS quando necessário. ULT continua somente notificando:
  não para motores nem bloqueia comandos dentro do driver.

Compatibilidade: `esp_drv8833_coast()` e o comando de velocidade zero em ambas
as rodas conservam a entrada em sleep quando EEP está configurado. As novas
APIs são aditivas; a estrutura pública de estado cresceu e consumidores devem
ser recompilados. Nenhum temporizador de frenagem foi introduzido no driver.

A aplicação precisa limitar a duração da frenagem e terminar em coast/sleep,
mesmo se os sensores falharem. Escritas de entrada são sequenciais, podendo
atravessar estados intermediários de drive; pulsos, desaceleração, corrente e
consumo exigem medição física. Erros de PWM/GPIO/API continuam separados da
telemetria ULT e não devem ser inferidos a partir do contexto temporal de ULT.

Validação: testes nativos com ASan/UBSan e compilação ESP-IDF 6.0 para C3/LEDC
e S3/MCPWM. Compilação e mocks não qualificam comportamento elétrico ou limites
de tempo; não substituem os ensaios físicos descritos no README.
