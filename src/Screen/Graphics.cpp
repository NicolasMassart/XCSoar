/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/
#include "Screen/Graphics.hpp"
#include "Screen/Point.hpp"
#include "Screen/UnitSymbol.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Bitmap.hpp"
#include "Screen/Brush.hpp"
#include "Screen/Color.hpp"
#include "Screen/Pen.hpp"
#include "Screen/Canvas.hpp"
#include "MapSettings.hpp"
#include "resource.h"
#include "Asset.hpp"
#include "LogFile.hpp"

Brush Graphics::hbGround;

const Color Graphics::GroundColor = Color(0x80,0x45,0x15);
const Color Graphics::skyColor = Color(0x0a,0xb9,0xf3);
const Color Graphics::seaColor = Color(0xbd,0xc5,0xd5); // ICAO open water area

void
Graphics::Initialise()
{
  /// @todo enhancement: support red/green color blind pilots with adjusted colour scheme

  LogStartUp(_T("Initialise graphics"));

  LoadUnitSymbols();

  hbGround.Set(GroundColor);
}

void
Graphics::Deinitialise()
{
  DeinitialiseUnitSymbols();

  hbGround.Reset();
}
