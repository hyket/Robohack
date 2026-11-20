# Unitree Go1 · T2

Самостоятельный образ Participant для задания T2. Среда внутри образа — Ubuntu 22.04, ROS 2 Humble и Cyclone DDS, платформа `linux/amd64`. Процесс запускается от пользователя `participant` с UID 10001.

## Сборка и локальная проверка

На Ubuntu с Docker и Buildx выполни из папки проекта:

```bash
bash t2_etap6.sh
```

Если репозиторий симулятора расположен в другом месте, передай путь:

```bash
bash t2_etap6.sh /путь/к/go1-simulator-public
```

Скрипт собирает локальный образ `go1-participant:t2`. Сборка использует интернет для получения образов и пакетов. В ходе проверки контейнер без сети выполняет `ping` и загружает модель Go1. Затем `run_local.py` поднимает два контейнера. Проверочный узел посылает цели в `/lowcmd_bare` и контролирует команды участника в `/lowcmd`. При успехе появляется `ЭТАП 6 ПРОЙДЕН`.

Отдельная сборка из этой же папки:

```bash
docker buildx build --platform linux/amd64 --load -t go1-participant:t2 .
```

Режимы входа в образ:

- `/entrypoint.sh ping` возвращает один JSON с заявленным T2.
- `/entrypoint.sh T2` запускает Safety Block на штатных ROS 2-топиках.

## Сдача

Добавь исходники из этой папки в публичный репозиторий команды. Можно положить их в отдельную папку `t2/`. Сохрани и отправь коммит, соответствующий проверенному образу. В личном кабинете укажи ссылку именно на этот коммит.

После локальной проверки опубликуй **тот же образ** в разделе Harbor, выделенном организаторами. Подставь назначенное имя команды в обоих сегментах пути:

```bash
docker login registry.cookcloud.ru
image=registry.cookcloud.ru/ИМЯ_КОМАНДЫ/ИМЯ_КОМАНДЫ:latest
docker tag go1-participant:t2 "$image"
docker push "$image"
docker buildx imagetools inspect "$image"
```

Из вывода возьми `Digest: sha256:…`. С этим значением проверь загрузку и передай ту же команду в личный кабинет:

```bash
docker pull --platform linux/amd64 registry.cookcloud.ru/ИМЯ_КОМАНДЫ/ИМЯ_КОМАНДЫ@sha256:ФАКТИЧЕСКИЙ_DIGEST
```

[Инструкция организаторов по сдаче](https://github.com/Urika000/go1-simulator-public/tree/main/docs)

## Происхождение зависимостей

Таблица проверяемых пар и `requirements.lock` взяты из go1-simulator-public, коммит `3473f086fd8a111592b4b9d822500c79f20c811a`. Лицензия включена в `NOTICE-go1-MIT.txt`.

Исходники `unitree_go` взяты из [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go). Модель Go1 соответствует [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/0059d4335f8156206f63a35662313385f7ad6d74/unitree_go1). Эти версии совпадают с конфигурацией публичного симулятора. Лицензии включены в образ.
