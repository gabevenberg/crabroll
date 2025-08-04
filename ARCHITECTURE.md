Crabroll is built on top of [Embassy](https://embassy.dev/) and [esp-hal](https://github.com/esp-rs/esp-hal).

It uses the embassy networking stack and [rust-mqtt](https://github.com/obabec/rust-mqtt) to manage communication with home assistant.
It uses a custom stepper motor driver (which will eventually be split into a standalone crate) to drive a stepper motor with a tmc2209 stepper motor driver.

Main spawns all tasks,
and does the inital hardware setup, as well as allocating resources and channels on the stack for communication between tasks.

Roughly, the architcture looks like:
```mermaid
flowchart
main[[main]]
main --o step_executor
main --o wifi_runner
main --o mqtt_listener & mqtt_sender
subgraph stepper motor
step_executor[[step executor]]-->step_planner[step planner]
tmc_configurator[TMC2209 configuration driver]
end
subgraph networking
wifi_runner[[wifi runner]]
mqtt_listener[[MQTT listener]]
mqtt_sender[[MQTT sender]]
end
mqtt_listener --x step_executor
step_executor --x mqtt_sender
main --> tmc_configurator
```

```mermaid
flowchart
subgraph legend
task[[task]] -- spawns --o task2[[task]]
task[[task]] -- communicates with --x task2[[task]]
task -- calls --> function[function]
end
```
