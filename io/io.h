/*
 * io_ex.h
 *
 *  Created on: Feb 26, 2025
 *      Author: admin
 *
 * Description: GPIO abstraction layer for STM32 LL with inversion support
 *
 * Note: The code assumes that Error_Handler() is defined elsewhere in the project.
 *       It must be implemented to handle errors appropriately, such as by
 *       resetting the system or entering an infinite loop.
 */

#ifndef APP_DRV_IO_EX_H_
#define APP_DRV_IO_EX_H_

#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED
#include "basic_types.h"
#include <cassert>

namespace gpio { // Added namespace to avoid naming conflicts

// Template class for digital input. Parameter Invert = true for inverted logic
template<bool Invert = false>
class DigitalInput {
public:
	DigitalInput() = default; // Default constructor

	explicit DigitalInput(GPIO_TypeDef* const port, const u32 pin)
	: gpio_port(port), gpio_pin(pin) {
		if (port == nullptr || !IS_GPIO_PIN(pin)) {
			Error_Handler(); // Call error handler for invalid parameters
		}
	}

	template<bool _T>
	explicit DigitalInput(const DigitalInput<_T>& other)
	: gpio_port(other.gpio_port), gpio_pin(other.gpio_pin) {
		if (other.gpio_port == nullptr || !IS_GPIO_PIN(other.gpio_pin)) {
			Error_Handler();
		}
	}

	DigitalInput& operator=(const DigitalInput& other) {
		gpio_port = other.gpio_port;
		gpio_pin = other.gpio_pin;

		if (other.gpio_port == nullptr || !IS_GPIO_PIN(other.gpio_pin)) {
			Error_Handler();
		}

		return *this;
	}

	/// @brief Checks if the input is properly initialized.
	/// @return True if the port is not nullptr, otherwise false.
	inline constexpr bool isInitialized() const {
		return gpio_port != nullptr;
	}

	/// @brief Reads the state of the input pin.
	/// @return True if the pin is active (high for normal logic, low for inverted).
	/// @note If the object is not initialized, calls Error_Handler() and returns false.
	inline constexpr bool read() const {
		if constexpr (Invert) {
			return LL_GPIO_IsInputPinSet(gpio_port, gpio_pin) == 0;
		} else {
			return LL_GPIO_IsInputPinSet(gpio_port, gpio_pin) != 0;
		}
	}

	inline constexpr operator bool() const { return read(); }

private:
	GPIO_TypeDef* gpio_port = nullptr;
	u32 gpio_pin = 0;
};

// Template class for digital output. Parameter Invert = true for inverted logic
template<bool Invert = false>
class DigitalOutput {
public:
	DigitalOutput() = default; // Default constructor

	explicit DigitalOutput(GPIO_TypeDef* const port, const u32 pin)
	: gpio_port(port), gpio_pin(pin) {
		if (port == nullptr || !IS_GPIO_PIN(pin)) {
			Error_Handler(); // Handle invalid parameters
		}
	}

	template<bool _T>
	explicit DigitalOutput(const DigitalOutput<_T>& other)
	: gpio_port(other.gpio_port), gpio_pin(other.gpio_pin) {
		if (other.gpio_port == nullptr || !IS_GPIO_PIN(other.gpio_pin)) {
			Error_Handler();
		}
	}

	DigitalOutput& operator=(const DigitalOutput& other) {
		gpio_port = other.gpio_port;
		gpio_pin = other.gpio_pin;

		if (other.gpio_port == nullptr || !IS_GPIO_PIN(other.gpio_pin)) {
			Error_Handler();
		}
		return *this;
	}

	/// @brief Checks if the input is properly initialized.
	/// @return True if the port is not nullptr, otherwise false.
	inline constexpr bool isInitialized() const {
		return gpio_port != nullptr;
	}

	/// @brief Writes the state to the output pin.
	/// @param state The state to write (true for active, false for inactive).
	/// @note For inverted logic, true sets the pin low, false sets it high.
	inline constexpr void write(const bool state) {
		if constexpr (Invert) {
			if (state) {
				LL_GPIO_ResetOutputPin(gpio_port, gpio_pin);
			} else {
				LL_GPIO_SetOutputPin(gpio_port, gpio_pin);
			}
		} else {
			if (state) {
				LL_GPIO_SetOutputPin(gpio_port, gpio_pin);
			} else {
				LL_GPIO_ResetOutputPin(gpio_port, gpio_pin);
			}
		}
	}

	/// @brief Reads the current logical state of the output.
	/// @return The logical state, consistent with write() and inversion logic.
	inline constexpr bool read() const {
		if constexpr (Invert) {
			return LL_GPIO_IsOutputPinSet(gpio_port, gpio_pin) == 0; // Invert physical state for logical state
		} else {
			return LL_GPIO_IsOutputPinSet(gpio_port, gpio_pin) != 0;
		}
	}

	/// @brief Reads the real logical state of the output.
	/// @return The logical state, consistent with write() and inversion logic.
	inline constexpr bool realRead() const {
		if constexpr (Invert) {
			return LL_GPIO_IsInputPinSet(gpio_port, gpio_pin) == 0;
		} else {
			return LL_GPIO_IsInputPinSet(gpio_port, gpio_pin) != 0;
		}
	}

	/// @brief Toggles the state of the output pin.
	inline constexpr void toggle() {
		LL_GPIO_TogglePin(gpio_port, gpio_pin);
	}

	inline constexpr DigitalOutput& operator=(const bool state) {
		write(state);
		return *this;
	}

	inline constexpr operator bool() const { return read(); }

private:
	GPIO_TypeDef* gpio_port = nullptr;
	u32 gpio_pin = 0;
};

// defining user types
using Output = DigitalOutput<false>;
using Output_n = DigitalOutput<true>;

using Input = DigitalInput<false>;
using Input_n = DigitalInput<true>;

} // namespace gpio

#endif // HAL_GPIO_MODULE_ENABLED
#endif // APP_DRV_IO_EX_H_
