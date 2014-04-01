#include "laser_icp_alg.h"

LaserIcpAlgorithm::LaserIcpAlgorithm(void)
{
}

LaserIcpAlgorithm::~LaserIcpAlgorithm(void)
{
}

void LaserIcpAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// LaserIcpAlgorithm Public API