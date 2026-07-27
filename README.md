# RoboTrio



## Структура файлов

| Файл | Назначение |
| :--- | :--- |
| `Dockerfile` | Содержит инструкции для сборки двух образов: `clean` (базовая система) и `base` (полное окружение с ROS 2, Gazebo и Python-зависимостями). |
| `docker-compose.yml` | Настройка для удобного запуска контейнеров с пробросом графики, GPU и монтированием папки с кодом. |
| `requirements.txt` | Список Python-пакетов, которые устанавливаются в образ `base`. |
| `README.md` | Этот файл. |

## Структура контейнеров

Все докер образы собраны в системе Container Registry от GitLab.

| Образ | Назначение |
| :--- | :--- |
| `clean` | "Чистый" контейнер, содержащий в себе систему Ububntu 24.04 с базовыми утилитами и ROS2 Jazzy. |
| `base` | Контейнер, содержащий в себе необходимые пакеты Python3 и программу Gazebo Jetty.


## Инструкция по запуску

Для работы с докером необходимо настроить `docker` на ПК. 

```sh
sudo apt-get update && sudo apt-get install -y gnome-keyring
wget -O docker-credential-secretservice https://github.com/docker/docker-credential-helpers/releases/download/v0.8.0/docker-credential-secretservice-v0.8.0.linux-amd64
chmod +x docker-credential-secretservice
sudo mv docker-credential-secretservice /usr/local/bin/

mkdir -p ~/.docker
cat > ~/.docker/config.json << EOF
{
  "credsStore": "secretservice"
}
EOF
```

Далее необходимо сгенерировать токен GitLab. `Профиль` -> `Access` -> `Personal Access Tokens` -> `Generate Token` -> `Legacy token`. Необходимо указать имя токена, дату истечения срока годности и выдать разрешение `read-registry`. Далее авторизируемся в докере на ПК.

```sh
echo <YOUR_TOKEN> | docker login registry.gitlab.com -u <USERNAME> --password-stdin
```

После этого нужно подключить свой git к GitLab, если ещё этого не делали:
```sh
git config --global credential.helper manager
```
Далее первая же команда, требующая авторизацию, запросит её.

Теперь можно скачивать репозиторий:

```sh
cd path/to/
git clone https://gitlab.com/aires_team/robotrio.git
```

Папка проекта проброшена внутрь докера, так что файлы создаваемые на хосте, будут видны внутри контейнеров.

Имя контейнера: `<CONTAINER_NAME>` = `robotrio-` + `<IMAGE_NAME>` 

Чтобы контейнер мог получить доступ к экрану выполните:

```sh
xhost +local:docker
```

Для запуска контейров используйте команду:

```sh
docker-compose run --rm <IMAGE_NAME> # запуск с привязкой к терминалу
docker-compose up -d <IMAGE_NAME> # запуск в фоновом (detach) режиме
``` 

Для подключения к уже запущенному контейнеру используйте:

```sh
docker-compose exec -it <IAMGE_NAME>
```

Для остановки контейнера, привязанному к терминалу необходимо либо выполнить команду `exit` внутри контейнера, либо закрыть терминал. Для остановки контейнера в detach-режиме выполните:

```sh
docker-compose stop <IMAGE_NAME>
```

**Важно!** Перед запуском система сама загрузит образ контейнера из удалённого репозитория, если не найдёт локальную версию. Если вы знаете, что удалённая версия норее и вам нужно обновить своюЮ, выполните команду:

```sh
docker-compose pull <IMAGE_NAME> 
```

Если вы внесли изменения локально и хотите их сохранить в проекте, у вас три варианта дейтсвий:

* Изменить существующую инструкцию создания образа в `Dockerfile`, добавив туда необходимые действия.
* По шаблону, создать новый образ через тот же `Dockerfile`.
* Запушить текущую версию образа на сервер напрямую. В таком случае, она заморозится как есть и не будет пересоздаваться при каждом коммите, но и воссоздать её с нуля будет невозможно. Для этого используйте:

```sh
docker commit -m "<MESSAGE>" <CONTAINER_ID> <NEW_NAME>:<TAG|latest>
docker push <YOUR_USERNAME>/<NEW_NAME>:<TAG|latest>
```

При работе с `Dockerfile` необходимо вносить изменения в `docker-compose.yml` и `.gitlab-ci.yml`. Уточнять у Ильи.
