# Технический стек Robohack

## Контейнер Participant

Контейнер поддерживает проверку допуска T1 и режим T3. Команда `/entrypoint.sh ping`
возвращает JSON протокола `1.0` с `supported_tasks=["T3"]`.

| Компонент | Текущая реализация |
| --- | --- |
| Образ | Docker/OCI, платформа `linux/amd64`, Ubuntu 22.04 LTS (Jammy) |
| Базовый образ | `ros:humble-ros-base-jammy`, закреплён по SHA-256 digest |
| Робототехнический фреймворк | ROS 2 Humble |
| Транспорт ROS 2 | Eclipse Cyclone DDS, пакет `ros-humble-rmw-cyclonedds-cpp`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Точка входа | Скрипт `/entrypoint.sh`, аргумент по умолчанию `ping`, режим `T3` запускает политику и Safety Block |
| Политика T3 | Walk These Ways для Go1, веса `.npz`, инференс на CPU в NumPy |
| Пользователь | `participant`, UID/GID `10001:10001` |
| Сборка и проверка | Docker Buildx и запуск `ping` без сети |

Конфигурация образа и точки входа описана в исходниках. [Dockerfile](../Dockerfile) · [entrypoint.sh](../entrypoint.sh)

Команды сборки, проверки и публикации приведены в README. [Сборка и проверка](../README.md#сборка-и-проверка) · [Публикация](../README.md#публикация)

## Среды испытаний T2–T5

| Задание | Стек по требованиям организаторов |
| --- | --- |
| T2, T3 | MuJoCo Lowlevel |
| T4 | Gazebo Classic Highlevel, lockstep, фронтальная RGB-D камера |
| T5 | Gazebo Classic Course, lockstep, CHAMP, Nav2 и пять RGB-D камер |

Во время испытаний Participant работает на CPU, без доступа к интернету. Все зависимости,
конфигурации и модели для заявленных режимов должны входить в образ. [Требования хакатона](HACKATHON.md)
