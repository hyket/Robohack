# Unitree Go1 · Participant

Контейнер Participant поддерживает T3 и проверку допуска T1. Образ рассчитан на `linux/amd64` и включает Ubuntu 22.04, ROS 2 Humble и Eclipse Cyclone DDS (`rmw_cyclonedds_cpp`). Контейнер работает с UID/GID `10001:10001`.

`/entrypoint.sh T3` запускает обученную авторами Walk These Ways политику Go1 и Safety Block из T2. Запуск публичного симулятора, запись видео и обучение описаны в [руководстве T3](T3/README.md). [Отдельный образ T2](T2/t2_checked/README.md) собирается из своего Dockerfile.

План разработки и проверки всех пяти заданий, связи между ними и порядок сдачи собраны в [руководстве по заочному этапу](Docs/GUIDE.md). Для изучения с нуля откройте в браузере [HTML-учебник](Docs/GUIDE.html). В нём есть введение перед каждым заданием, схемы обмена между узлами и вопросы для самопроверки.

## Сборка и проверка

Сборка загружает зависимости ROS 2 и MuJoCo. Для запуска все зависимости и веса уже находятся внутри образа.

```sh
docker buildx build --platform linux/amd64 --load -t go1-participant:t3 .
docker image inspect --format '{{.Os}}/{{.Architecture}} {{.Config.User}}' go1-participant:t3
time docker run --rm --pull never --platform linux/amd64 --network none \
  --cap-drop ALL --security-opt no-new-privileges \
  --entrypoint /entrypoint.sh go1-participant:t3 ping
```

Ожидаемый ответ с кодом завершения `0` за 10 секунд или быстрее.

```json
{"protocol_version":"1.0","supported_tasks":["T3"],"self_trained_rl":false,"preferred_speed_mps":0.6}
```

## Публикация

Опубликуйте этот коммит в публичном репозитории команды. Задайте `TEAM_NAME` именем проекта, которое назначили организаторы в Harbor, и выполните команды из каталога с Dockerfile.

```sh
docker login registry.cookcloud.ru
IMAGE_REPO="registry.cookcloud.ru/${TEAM_NAME:?Set TEAM_NAME}/$TEAM_NAME"
docker buildx build --platform linux/amd64 --provenance=false \
  --tag "$IMAGE_REPO:latest" --push .
docker buildx imagetools inspect "$IMAGE_REPO:latest"
```

Скопируйте значение `Digest` в `DIGEST`, включая префикс `sha256:`. Проверьте загрузку по digest и передайте эту команду вместе со ссылкой на конкретный Git-коммит в личном кабинете. [Инструкция организаторов](https://github.com/Urika000/go1-simulator-public/blob/main/docs/Инструкция_по_загрузке_решения.md)

```sh
docker pull --platform linux/amd64 "$IMAGE_REPO@${DIGEST:?Set DIGEST}"
```
