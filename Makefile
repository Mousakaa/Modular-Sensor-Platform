default:
	@echo "Please specify what you want to make (\`build-ui\` or \`flash\`)"

build-ui:
	@cd mspview; cargo build --release
	@cp mspview/target/release/mspview -t ~/.local/bin
	@echo "UI built successfully, type \`mspview\` to start it."

flash:
ifdef esp_idf_path
	@cd mspsensor; source $(esp_idf_path)/export.sh; idf.py menuconfig build flash
else
	@echo "Please provide path to ESP-IDF folder with \`make flash esp_idf_path=path/to/esp-idf\`"
	@echo "If it is not installed, you can find it at https://github.com/espressif/esp-idf"
endif
