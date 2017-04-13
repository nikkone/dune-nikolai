//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Paulo Dias                                                       *
// Author: Pedro Calado (Implementation of original Rows maneuver)          *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Maneuver
{
  namespace Docking
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      // System id
      const char* announce_id;
      double lat;
      double lon;

    };


    const char* docking_id;

    struct Task: public DUNE::Maneuvers::Maneuver
    {
      //! Maneuver
      IMC::Docking m_maneuver;

      IMC::Announce m_announce;

      //! Task arguments
      Arguments m_args;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx)
      {
        bindToManeuver<Task, IMC::Docking>();
        bind<IMC::Announce>(this);
      }

      //! Destructor
      virtual
      ~Task(void)
      {
      }

      void
      onManeuverDeactivation(void)
      {
      }

      void
      consume(const IMC::Docking* maneuver)
      {
        if (maneuver->getSource() != getSystemId())
          return;
        m_maneuver = *maneuver;
      }

      void
      consume(const IMC::Announce* msg)
      {
        m_args.announce_id = msg->sys_name.c_str();
        docking_id = m_maneuver.target.c_str();
        if (strcmp(m_args.announce_id , docking_id) == 0)
        {
          war("\n\nVehicle found\n");
          war("%s",m_args.announce_id);
          war("%s",docking_id);
          m_args.lat = msg->lat;
          m_args.lon = msg->lon;
          war("lat vehicle: %f", m_args.lat);
          war("lon vehicle: %f", m_args.lon);
          IMC::DesiredPath path;
          setControl(IMC::CL_PATH);
          path.end_lat = m_args.lat;
          path.end_lon = m_args.lon;
          path.speed_units = IMC::SUNITS_METERS_PS;
          path.speed = 1;
          dispatch(path);
        }
        else
          war("\n\nVehicle nowhere found\n");
      }

    };
  }
}

DUNE_TASK
