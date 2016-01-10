
#define LOW 0
#define HIGH 1

int get_gpio_value(int gpio_nr);
int set_gpio_value(int gpio_nr, int value);
int set_gpio_input(int gpio_nr);
int set_gpio_output(int gpio_nr, int value);
int unexport_gpio(int gpio_nr);
int export_gpio(int gpio_nr);
