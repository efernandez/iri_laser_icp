#include "laser_icp_alg.h"

LaserIcpAlgorithm::LaserIcpAlgorithm(void)
{
    pthread_mutex_init(&this->access_,NULL);
}

LaserIcpAlgorithm::~LaserIcpAlgorithm(void)
{
    pthread_mutex_destroy(&this->access_);
}

void LaserIcpAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// LaserIcpAlgorithm Public API