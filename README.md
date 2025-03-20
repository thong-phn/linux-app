# Create the application folder
```shell
git clone https://github.com/thong-phn/temperature_prediction_zephyr temp_prediction
```
# Build the application for qemu_xtensa
```shell
west build -p always -b qemu_xtensa temp_prediction
```
# Run the application on qemu_xtensa
```shell
west build -t run
```