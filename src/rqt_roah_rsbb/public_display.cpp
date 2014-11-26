/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB.
 *
 * RoAH RSBB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "public_display.h"

#include <QStringList>
#include <QDesktopWidget>

#include <roah_utils.h>
#include <ros_roah_rsbb.h>



using namespace std;
using namespace ros;



PublicDisplay::PublicDisplay()
: nh_()
, screen_srv_ (nh_.advertiseService ("screen", &PublicDisplay::set_screen, this))
{
  setObjectName ("PublicDisplay");
  
  ui_.setupUi (this);
  
  // core_rcv_.start ("/core/to_gui", getNodeHandle());
  
  connect (&update_timer_, SIGNAL (timeout()), this, SLOT (update()));
  update_timer_.start (1000);
  
  int numScreens = QApplication::desktop()->numScreens();
  if(numScreens > 1){
  cerr << "------------------------------------------------------------------------" << endl;
  for(int i = 0 ; i < numScreens ; ++i){
  cerr << "rosservice call " << screen_srv_.getService() << " " << i << endl;
  }
  cerr << "------------------------------------------------------------------------" << endl << flush;
  }
}


PublicDisplay::~PublicDisplay()
{
  update_timer_.stop();
  core_rcv_.stop();
  shutdown();
}

bool PublicDisplay::set_screen(roah_rsbb::UInt8::Request& req,
                               roah_rsbb::UInt8::Response& res){
  QRect screenres = QApplication::desktop()->screenGeometry(req.data);
  move(QPoint(screenres.x(), screenres.y()));
  resize(screenres.width(), screenres.height());
  showFullScreen();
  
  return true;
}

void PublicDisplay::update()
{
  spinOnce();
  if(! ok()){
  QApplication::quit();
  }
  
  
}
