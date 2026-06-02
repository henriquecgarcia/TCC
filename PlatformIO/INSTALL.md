# INSTALL.md — Instalação e execução

## 1. Abrir no VSCode

1. Extraia o `.zip`.
2. Abra a pasta no VSCode.
3. Instale a extensão PlatformIO.
4. Aguarde o PlatformIO carregar o ambiente.

## 2. Compilar

```bash
pio run
```

## 3. Enviar firmware

A porta atual no `platformio.ini` está configurada como `COM3`.

```bash
pio run -t upload
```

## 4. Enviar SPIFFS

A interface Web fica em `/data` e precisa ser enviada para a flash.

```bash
pio run -t uploadfs
```

## 5. Monitor serial

```bash
pio device monitor -b 115200
```

## 6. Acessar interface

Após conectar no Wi-Fi configurado em `src/carrinho.ino`, acesse o IP mostrado no monitor serial.

Rotas úteis:

```txt
/                  Painel principal LCARS
/brownout          Painel de brownout
/api/tcc-log.csv   Download do log experimental
```

## 7. Comandos principais

```json
{"action":"forward_cells","cells":3}
```

```json
{"action":"path_to","x":8,"y":4,"execute":true}
```

```json
{"action":"experiment_log","mode":"clear"}
```

