/*
*  Project     Nintendo Extension Controller Library
*  @author     David Madison
*  @link       github.com/dmadison/NintendoExtensionCtrl
*  @license    LGPLv3 - Copyright (c) 2018 David Madison
*
*  This file is part of the Nintendo Extension Controller Library.
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "GuitarController.h"

namespace NintendoExtensionCtrl {

constexpr ByteMap GuitarController_Shared::Maps::JoyX;
constexpr ByteMap GuitarController_Shared::Maps::JoyY;

constexpr BitMap  GuitarController_Shared::Maps::ButtonPlus;
constexpr BitMap  GuitarController_Shared::Maps::ButtonMinus;

constexpr BitMap  GuitarController_Shared::Maps::StrumUp;
constexpr BitMap  GuitarController_Shared::Maps::StrumDown;

constexpr BitMap  GuitarController_Shared::Maps::FretGreen;
constexpr BitMap  GuitarController_Shared::Maps::FretRed;
constexpr BitMap  GuitarController_Shared::Maps::FretYellow;
constexpr BitMap  GuitarController_Shared::Maps::FretBlue;
constexpr BitMap  GuitarController_Shared::Maps::FretOrange;

constexpr ByteMap GuitarController_Shared::Maps::Whammy;
constexpr ByteMap GuitarController_Shared::Maps::Touchbar;

uint8_t GuitarController_Shared::joyX() const {
	return getControlData(Maps::JoyX);
}

uint8_t GuitarController_Shared::joyY() const {
	return getControlData(Maps::JoyY);
}

bool GuitarController_Shared::strum() const {
	return strumUp() | strumDown();
}

bool GuitarController_Shared::strumUp() const {
	return getControlBit(Maps::StrumUp);
}

bool GuitarController_Shared::strumDown() const {
	return getControlBit(Maps::StrumDown);
}

bool GuitarController_Shared::fretGreen() const {
	return getControlBit(Maps::FretGreen);
}

bool GuitarController_Shared::fretRed() const {
	return getControlBit(Maps::FretRed);
}

bool GuitarController_Shared::fretYellow() const {
	return getControlBit(Maps::FretYellow);
}

bool GuitarController_Shared::fretBlue() const {
	return getControlBit(Maps::FretBlue);
}

bool GuitarController_Shared::fretOrange() const {
	return getControlBit(Maps::FretOrange);
}

uint8_t GuitarController_Shared::whammyBar() const {
	return getControlData(Maps::Whammy);
}

uint8_t GuitarController_Shared::touchbar() const {
	return getControlData(Maps::Touchbar);
}

bool GuitarController_Shared::touchGreen() const {
	return touchbar() != 0 && touchbar() <= 7;
}

bool GuitarController_Shared::touchRed() const {
	return touchbar() >= 7 && touchbar() <= 13;
}

bool GuitarController_Shared::touchYellow() const {
	return touchbar() >= 12 && touchbar() <= 21
		&& touchbar() != 15;	// The "not touched" value
}

bool GuitarController_Shared::touchBlue() const {
	return touchbar() >= 20 && touchbar() <= 26;
}

bool GuitarController_Shared::touchOrange() const {
	return touchbar() >= 26;
}

bool GuitarController_Shared::buttonPlus() const {
	return getControlBit(Maps::ButtonPlus);
}

bool GuitarController_Shared::buttonMinus() const {
	return getControlBit(Maps::ButtonMinus);
}

bool GuitarController_Shared::supportsTouchbar() {
	if (touchbarData) {
		return true;
	}
	else if (touchbar() != 31) {  // '1' for all data bits
		touchbarData = true;
	}
	return false;
}


}  // End "NintendoExtensionCtrl" namespace
