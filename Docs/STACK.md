# Технический стек Robohack

## Контейнер Participant

Текущая реализация покрывает T1. Контейнер поддерживает команду `/entrypoint.sh ping`,
которая возвращает JSON протокола `1.0` с пустым `supported_tasks`.

| Компонент | Текущая реализация |
| --- | --- |
| Образ | Docker/OCI, платформа `linux/amd64`, Ubuntu 22.04 LTS (Jammy) |
| Базовый образ | `ros:humble-ros-base-jammy`, закреплён по SHA-256 digest |
| Робототехнический фреймворк | ROS 2 Humble |
| Транспорт ROS 2 | Eclipse Cyclone DDS, пакет `ros-humble-rmw-cyclonedds-cpp`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Точка входа | POSIX `sh`, скрипт `/entrypoint.sh`, аргумент по умолчанию `ping` |
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
