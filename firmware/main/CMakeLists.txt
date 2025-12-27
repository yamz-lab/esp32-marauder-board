# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c" 
    	 "bme280/i2c_bme280.c"		# list the source files of this component
    	 "lcd_driver/lcd_driver.c"
    	 "gui/gui.c"
    	 "menu/menu.c"
    	 "wifi/wifi.c"
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES	driver
    			esp_wifi
    			nvs_flash
    			# optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)
