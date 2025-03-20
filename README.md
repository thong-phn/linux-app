`
# Create the application folder
mkdir temp_prediction
git clone https://github.com/thong-phn/temperature_prediction_zephyr
# Build the application for qemu_xtensa
west build -p always -b qemu_xtensa temp_prediction
# Run the application on qemu_xtensa
west build -t run
`