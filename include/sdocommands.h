/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2025 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SDOCOMMANDS_H
#define SDOCOMMANDS_H
#include "cansdo.h"
#include "canmap.h"

class SdoCommands
{
   public:
      static void ProcessStandardCommands(CanSdo::SdoFrame* sdoFrame);
      static void EnableSaving() { saveEnabled = true; }
      static void DisableSaving() { saveEnabled = false; }
      static void SetCanMap(CanMap* m) { canMap = m; }

   private:
      static CanMap* canMap;
      static bool saveEnabled;
};

#endif // SDOCOMMANDS_H
