# Documentación de ramas y guía de contribución — crazyflie-firmware

Esta rama (`doc`) no contiene código: solo explica para qué sirve cada rama de interés y cómo contribuir en este fork.

## Ramas de este repositorio

| Rama | Propósito |
|---|---|
| `master` | Fork de [`bitcraze/crazyflie-firmware`](https://github.com/bitcraze/crazyflie-firmware), mantenido al día con upstream. Personalizaciones del laboratorio bajo `examples/` (`multi_robot_system`, `app_p2p_DTR`, `gimbal`, `event_based_controller`…) y comandos CRTP añadidos en `src/modules/src/crtp_commander_high_level.c` (IDs 14-24: formación, relay, comandos multi-agente — coordinados con `crazyflie-lib-python`). |
| `doc` (esta) | Documentación de ramas y guía de contribución. |
| (resto de ramas) | Ramas de trabajo/PR de Bitcraze traídas junto con el fork; no son del laboratorio. |

## Cómo sincronizar con Bitcraze

**Nunca se hace push a `bitcraze/crazyflie-firmware`.** El flujo es de un solo sentido:
1. Traer los cambios de `bitcraze/crazyflie-firmware@master`.
2. Si un comando o ID CRTP propio colisiona con uno nuevo de Bitcraze (ya ha pasado una vez, ver commit de sincronización de 2026-08), renumerar el propio y actualizar `crazyflie-lib-python` en el mismo cambio — nunca al revés.
3. Publicar el resultado en una rama de revisión (p. ej. `sync-bitcraze`) antes de fusionar a `master`, y **probar en hardware/simulación** — un cambio de protocolo mal resuelto falla en silencio, no en compilación.

## Guía de contribución

- Antes de un PR: compila el firmware para el objetivo relevante (`make PLATFORM=cf2` u objetivo equivalente).
- Coordina cualquier ID de comando CRTP nuevo con `crazyflie-lib-python`.
- Actualiza esta página si cambia el propósito de alguna rama.

### Licencia
Este fork mantiene la licencia original de Bitcraze (GPL-3.0, ver `LICENSE.txt` en `master`).
