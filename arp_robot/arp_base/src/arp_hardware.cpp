#include "arp_base/arp_hardware.h"
#include <boost/assign/list_of.hpp>
#include "arp_base/roboteq_driver/controller.h"
#include "arp_base/roboteq_driver/channel.h"

namespace
{
const double RAD_STEP=1.308996939;

}

namespace arp_base
{

/**
 * @brief ArpHardware::ArpHardware
 * @param nh
 * @param private_nh
 * Inicilaliza o hardware do ARP
 */
ArpHardware::ArpHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<double>("polling_timeout_", polling_timeout_, 0.15);

  std::string port[NUM_CONTROLLERS];
  int32_t baund[NUM_CONTROLLERS];

  // inicializa os controladores individualmente com os paramtros do ROS
  for (int i = 0; i < NUM_CONTROLLERS; i++)
  {
    private_nh_.param<std::string>(
        "port_" + boost::lexical_cast<std::string>(i), port[i],
        "/dev/ttyACM" + boost::lexical_cast<std::string>(i));
    private_nh_.param<int32_t>("baund_" + boost::lexical_cast<std::string>(i),
                               baund[i], 115200);
    if (i % 2)
    {
      controller_[i].controlerInit(port[i].c_str(), baund[i], "left");
    }
    else
    {
      controller_[i].controlerInit(port[i].c_str(), baund[i], "right");
    }
    controller_[i].startScript();
    setupChannel(i);
    connect(i, port[i].c_str());
  }
  resetTravelOffset();
  registerControlInterfaces();
  initializeStatus();
}

/**
 * Para os motores e encerra o script ao destruir a classe
*/
ArpHardware::~ArpHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS; i++)
  {
    controller_[i].setEstop();
    controller_[i].stopScript();
  }
}

/**
 * Pega os dados coletados pelo driver Roboteq,
 * Calcula o delsocamento com base no offset aplicado na inicialização
 * Em caso de um deslocamento ecessivo o controle zera o offset
 * Os dados de velocidade também são coletados e passados para o ROS
 * Como cada controlador controla dois motores os parametros pares
 * Vão para o motor da frente e os impares para o motor de traz
*/
void ArpHardware::updateJointsFromHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    double delta =
        controller_[i/2].getChanels()[i % 2]->getFeedBack().measured_position -
        joints_[i].position - joints_[i].position_offset;

    // detecta perda de dados do encoder
    if (std::abs(delta) < 1.0)
    {
      joints_[i].position += delta;
    }
    else
    {
      joints_[i].position_offset += delta;
      ROS_DEBUG("Dropping overflow measurement from encoder");
    }
    joints_[i].velocity = controller_[i/2].getChanels()[i % 2]->getFeedBack().measured_velocity;
  }
}

/**
 * Envia os  comandos de velocidade para o controlador
 * Como cada controlador controla dois motores os parametros pares
 * Vão para o motor da frente e os impares para o motor de traz
*/
void ArpHardware::writeCommandsToHardware()
{
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    controller_[i/2].getChanels()[i%2]->cmdCallback(0, velocityDiscretizationFromController(joints_[i].velocity_command));
  }
}

/**
 * Registra os controles na interface do ROS
 * Os parametros são correspondentes da struct joint declarada no .h
*/
void ArpHardware::registerControlInterfaces()
{
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")(
      "back_left_wheel")("front_right_wheel")("back_right_wheel");
  for (int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[i], &joints_[i].position, &joints_[i].velocity,
        &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

/**
 * Obtem os valores de offset na icialização do dispositivo
 * Como cada controlador controla dois motores os parametros pares
 * Vão para o motor da frente e os impares para o motor de traz
 */
void ArpHardware::resetTravelOffset()
{
  for (int i = 0; i < NUM_CONTROLLERS * 2; i++)
  {
    joints_[i].position_offset =
        controller_[i/2].getChanels()[i % 2]->getFeedBack().measured_position;
  }
}

/**
 * Está rotina discretiza os valores de velocidade para multiplos de 12,5 RPM
 * isto acontece devido a baixa resolução dos encoders, retornando para o controle velocidades
 * multiplas de 12,5 RPM. Com isso o controlador não consegue controlar velocidades entre esse valor devido
 * ao erro gerado no PID.
*/
double ArpHardware::velocityDiscretizationFromController(double velocity)
{
  int velocity_ = velocity/RAD_STEP;
  return velocity_ * RAD_STEP;
}

/**
 * Inicializa dois canais por controlador, visto que cada controlador opera dois motores
*/
void ArpHardware::setupChannel(int index)
{
  controller_[index].addChannel(new roboteq::Channel(
      1, "front_" + boost::lexical_cast<std::string>(controller_[index].getControllerName()) + "_wheel",
      &controller_[index],polling_timeout_));
  controller_[index].addChannel(new roboteq::Channel(
      2, "back_" + boost::lexical_cast<std::string>(controller_[index].getControllerName()) + "_wheel",
      &controller_[index],polling_timeout_));
}

/**
 * Conecta os controladores com suas respectivas portas serias,
 * em caso de insucesso o sistema entra em um loop para forçar a conexão
*/
void ArpHardware::connect(int index, const char* port)
{
  ROS_DEBUG("Attempting connection to %s for %s wheels", port, controller_[index].getControllerName().c_str());

  while (ros::ok())
  {
    controller_[index].connect();

    if (controller_[index].connected())
    {
      ROS_DEBUG("Connection successful to %s for %s wheels", port, controller_[index].getControllerName().c_str());
      ROS_INFO("Connection successful to %s for %s wheels", port, controller_[index].getControllerName().c_str());
      break;
    }
    else
    {
      ROS_DEBUG("Problem connecting to serial device.");
      ROS_ERROR_STREAM_ONCE("Problem connecting to "
                            << controller_[index].getControllerName().c_str()
                            << " controller.Try again in 1 second.");
      sleep(1);
    }
  }
}

/**
 * Rotina de leitura do driver Roboteq, caso conectado ele executa a rotina
*/
void ArpHardware::initializeReadFromHardware(int index)
{
  if (controller_[index].connected())
  {
    controller_[index].spinOnce();
  }
  else
  {
    ROS_DEBUG("Problem connecting to serial device.");
    ROS_ERROR_STREAM_ONCE("Problem connecting to serial device.");
  }
}

/**
 * Atualiza o topico de status com os valores coletados do driver Roboteq
*/
void ArpHardware::updateStatus()
{
  arp_msgs::ArpStatus arp_status_msg;
  arp_status_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < NUM_CONTROLLERS; i++) {
    arp_status_msg.controller_name.push_back(controller_[i].getControllerName());
    arp_status_msg.fault.push_back(controller_[i].getStatus().fault);
    arp_status_msg.status.push_back(controller_[i].getStatus().status);
    arp_status_msg.internal_voltage.push_back(controller_[i].getStatus().internal_voltage);
    arp_status_msg.adc_voltage.push_back(controller_[i].getStatus().adc_voltage);
    for (int f = 0; f < 2; f++) {
      arp_status_msg.channel_name.push_back(controller_[i].getChanels()[f]->getName());
      arp_status_msg.motor_current.push_back(controller_[i].getChanels()[f]->getFeedBack().motor_current);
      arp_status_msg.motor_power.push_back(controller_[i].getChanels()[f]->getFeedBack().motor_power);
      arp_status_msg.commanded_velocity.push_back(controller_[i].getChanels()[f]->getFeedBack().commanded_velocity);
      arp_status_msg.supply_voltage.push_back(controller_[i].getChanels()[f]->getFeedBack().supply_voltage);
      arp_status_msg.supply_current.push_back(controller_[i].getChanels()[f]->getFeedBack().supply_current);
      arp_status_msg.channel_status.push_back(controller_[i].getChanels()[f]->getFeedBack().channel_status);
      arp_status_msg.motor_temperature.push_back(controller_[i].getChanels()[f]->getFeedBack().motor_temperature);
      arp_status_msg.channel_temperature.push_back(controller_[i].getChanels()[f]->getFeedBack().channel_temperature);
    }
  }
  status_publisher_.publish(arp_status_msg);
}

/**
 * Inicializa o topico status
*/
void ArpHardware::initializeStatus()
{
  status_publisher_ = nh_.advertise<arp_msgs::ArpStatus>("status",10);
}


}
