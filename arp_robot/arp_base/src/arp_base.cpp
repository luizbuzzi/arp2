#include "arp_base/arp_hardware.h"
#include "ros/callback_queue.h"
#include "controller_manager/controller_manager.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(arp_base::ArpHardware& arp,
                 controller_manager::ControllerManager& cm,
                 time_source::time_point& last_time)
{
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  arp.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  arp.writeCommandsToHardware();
}

/*
 * Loop de diagnóstico
*/
void diagnosticLoop(arp_base::ArpHardware& arp)
{
  arp.updateStatus();
}

/*
 * Loop de leitura pra os controladores Robotec
*/
void readController(arp_base::ArpHardware& arp, int& index)
{
  while (ros::ok())
  {
    arp.initializeReadFromHardware(index);
  }
}

int main(int argc, char* argv[])
{
  // Inicializa o nó no ros
  ros::init(argc, argv, "arp_base");
  ros::NodeHandle nh("~"), private_nh("~");

  // Obetem os parametros postos na inicialização do nó
  double control_frequency;
  double diagnostic_frequency;

  private_nh.param<double>("control_frequency", control_frequency, 10);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1);

  // Inicializa o hardware do ARP e cria um link com o controller manager
  arp_base::ArpHardware arp(nh, private_nh);
  controller_manager::ControllerManager cm(&arp, nh);

  // Inicializa uma thread para cada controlador, esta responsavel por
  // receber o fluxo de dados do hardware
  int controlers[arp.NUM_CONTROLLERS];

  for (int i = 0; i < arp.NUM_CONTROLLERS; i++)
  {
    controlers[i] = i;
    boost::thread read_controlers(boost::bind(readController, boost::ref(arp),
                                              boost::ref(controlers[i])));
  }

  // Inicializa uma fila de threads separada das rotina do ros
  // utiliza apenas uma thread para nao lidar com problemas de mutiplos acessos
  ros::CallbackQueue arp_queue;
  ros::AsyncSpinner arp_spinner(1, &arp_queue);

  // Loop de controle, disparado na frequencia definida pelo usuario
  // executa as rotinas da funçao controlLoop
  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(ros::Duration(1 / control_frequency),
                                  boost::bind(controlLoop, boost::ref(arp),
                                              boost::ref(cm),
                                              boost::ref(last_time)),
                                  &arp_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  // Loop de diagnostico, disparado na frequencia definida pelo usuario
  // executa as rotinas da funçao diagnosticLoop
  ros::TimerOptions diagnostic_timer(
      ros::Duration(1 / diagnostic_frequency),
      boost::bind(diagnosticLoop, boost::ref(arp)), &arp_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  arp_spinner.start();

  // Processa as chamadas do ROS separadamente
  ros::spin();

  return 0;
}
