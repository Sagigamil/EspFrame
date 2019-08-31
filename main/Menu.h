#pragma once
#include <driver/gpio.h>
#include "FreeRTOS.h"

class Menu
{
public:
	Menu(gpio_num_t pin,
		 uint32_t next_item_click_timeout,
		 uint32_t choose_item_click_timeout);

	void init();

	using display_item = void();
	using item_choosed = void();
	size_t add_menu_item(display_item *display_function, item_choosed *choose_function);

	size_t start();

private:
	
	static void isr_handler(void * param);
	static void task(void * param);

	struct MenuItem {
		display_item *display_func;
		item_choosed *choose_func;
	};

	MenuItem m_items[10];
	size_t m_number_of_items;
	size_t m_current_item;

	const gpio_num_t m_pin;
	const uint32_t m_next_item_click_timeout;
	const uint32_t m_choose_item_click_timeout;

	FreeRTOS::EventFlags m_flags;
	FreeRTOS::EventFlags m_end_task;
};