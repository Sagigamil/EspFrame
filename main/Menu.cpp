#include "Menu.h"

#define MENU__BUTTON_CLICKED 1
#define MENU__BUTTON_NOT_CLICKED 2
#define MENU__END 1

Menu::Menu(gpio_num_t pin,
		   uint32_t next_item_click_timeout,
		   uint32_t choose_item_click_timeout) : m_number_of_items(0),
												 m_current_item(0),
												 m_next_item_click_timeout(next_item_click_timeout),
												 m_choose_item_click_timeout(choose_item_click_timeout),
												 m_pin(pin) {}

void Menu::init()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = 1 << m_pin;
	gpio_config(&io_conf);

	gpio_install_isr_service(0);

	gpio_isr_handler_add(m_pin, &Menu::isr_handler, (void *)this);
}

size_t Menu::add_menu_item(display_item *display_function, item_choosed *choose_function)
{
	m_items[m_number_of_items].display_func = display_function;
	m_items[m_number_of_items].choose_func = choose_function;
	m_number_of_items++;

	return m_number_of_items - 1;
}

void Menu::isr_handler(void *param)
{
	Menu *menu = (Menu *)param;
	if (gpio_get_level(menu->m_pin))
	{
		menu->m_flags.set(MENU__BUTTON_CLICKED);
	}
	else
	{
		menu->m_flags.set(MENU__BUTTON_NOT_CLICKED);
	}
}

size_t Menu::start()
{
	FreeRTOS::startTask(task, "menu_task", this);
	m_end_task.wait(MENU__END);

	return m_current_item;
}

void Menu::task(void *param)
{
	Menu *menu = (Menu *)param;

	menu->m_items[menu->m_current_item].display_func();

	while (true)
	{
		menu->m_flags.wait(MENU__BUTTON_CLICKED);
		/* Now button is clicked */
		if (menu->m_flags.wait(MENU__BUTTON_NOT_CLICKED, true, true, menu->m_choose_item_click_timeout))
		{
			/* Button clicked for less then `m_choose_item_click_timeout`, skip to next item. */
			menu->m_current_item = (menu->m_current_item + 1) % menu->m_number_of_items;

			menu->m_items[menu->m_current_item].display_func();
		}
		else
		{
			/* Button still clicked, wait until not clicked */
			menu->m_flags.wait(MENU__BUTTON_NOT_CLICKED);
			/* Now call the click callback */
			menu->m_items[menu->m_current_item].choose_func();
			break;
		}
	}

	menu->m_end_task.set(MENU__END);
	vTaskDelete(NULL);
}

// size_t Menu::add_menu_item(display_item display_function, item_choosed);
