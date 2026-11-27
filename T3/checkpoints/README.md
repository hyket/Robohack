# Происхождение весов T3

`walk_these_ways.npz` содержит опубликованные предобученные веса для Unitree Go1, выпущенные Gabriel B. Margolis и Pulkit Agrawal вместе с работой *Walk These Ways: Tuning Robot Control for Generalization with Multiplicity of Behavior* (CoRL 2022). Исходная версия закреплена на коммите `0e7236bdc81ce855cbe3d70345a7899452bdeb1c`, запуск `runs/gait-conditioned-agility/pretrain-v0/train/025417.456545`. В этом репозитории веса преобразованы в NumPy без обучения и изменения параметров. [Исходный запуск](https://github.com/Improbable-AI/walk-these-ways/tree/0e7236bdc81ce855cbe3d70345a7899452bdeb1c/runs/gait-conditioned-agility/pretrain-v0/train/025417.456545) · [Статья](https://openreview.net/forum?id=52c5e73SlS2) · [Лицензия MIT](LICENSE-walk-these-ways.txt).

| Файл | SHA-256 |
| --- | --- |
| `adaptation_module_latest.jit` | `f0adb113954058a37d2596282f041a3bb128365558ecf64405bbc1353d5dc1b0` |
| `body_latest.jit` | `6c14b59ca28550a012de3bf5d21fc7d6aa6b6d38eab4dc31e172519f12a19ece` |
| `walk_these_ways.npz` | `e5daff12de8b5b0e7d105b48b8fb0bfb62f7fa3349c2b41ee964dfaee3507a2b` |

Команда ниже повторяет преобразование из корня Robohack. Для неё нужны NumPy и PyTorch с поддержкой CPU.

```bash
python -m T3.checkpoints.convert /path/to/025417.456545/checkpoints T3/checkpoints/walk_these_ways.npz
```

Преобразователь сравнивает NumPy и исходный TorchScript на 129 входных историях. Для приведённого файла максимальная абсолютная разница составила `1.37e-4` для выхода сети адаптации и `2.21e-4` для нормированного действия. При масштабе действия 0,25 это соответствует разнице целей суставов не более `5.53e-5` рад на проверенных входах. Для инференса нужен только NumPy.

Каждые 20 мс модель получает очередной кадр из 70 значений. Кадр содержит вектор гравитации в системе корпуса, 15 масштабированных параметров команды, 12 смещений углов от нейтральной позы, 12 скоростей суставов, два предыдущих действия по 12 значений и четыре сигнала фазы шага. История из 30 кадров хранится от старого кадра к новому. Сеть адаптации формирует два латентных значения. Сеть управления читает историю и эти значения и возвращает 12 нормированных действий. Порядок модели `FL, FR, RL, RR`, а порядок интерфейса Unitree `FR, FL, RR, RL`. В каждой ноге используются `hip, thigh, calf`. [Подготовка входа авторов](https://github.com/Improbable-AI/walk-these-ways/blob/0e7236bdc81ce855cbe3d70345a7899452bdeb1c/go1_gym_deploy/envs/lcm_agent.py) · [Загрузка модели авторами](https://github.com/Improbable-AI/walk-these-ways/blob/0e7236bdc81ce855cbe3d70345a7899452bdeb1c/go1_gym_deploy/scripts/deploy_policy.py).

Для MuJoCo организаторов позиционное усиление увеличено с 20 до 40, а `kd` равно 0,5. При подборе использовались исходная модель симулятора и фильтр T2. Регулятор применяет эти коэффициенты после инференса.
