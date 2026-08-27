#pragma once
#ifndef ROTATOR_API_H
#define ROTATOR_API_H

#include "RotatorHW.h"
#include "Version.h"
#include "alpaca_server/api.h"
#include "alpaca_server/discovery.h"

class RotatorApi : public AlpacaServer::Rotator
{
  public:
    RotatorApi(RotatorHW &rot);
    ~RotatorApi();

  public:
    virtual esp_err_t action(const char *action, const char *parameters, char *buf,
                             size_t len) override;
    virtual esp_err_t commandblind(const char *command, bool raw) override;
    virtual esp_err_t commandbool(const char *command, bool raw, bool *resp) override;
    virtual esp_err_t commandstring(const char *action, bool raw, char *buf, size_t len) override;
    virtual esp_err_t get_connected(bool *connected) override;
    virtual esp_err_t set_connected(bool connected) override;
    virtual esp_err_t get_description(char *buf, size_t len) override;
    virtual esp_err_t get_driverinfo(char *buf, size_t len) override;
    virtual esp_err_t get_driverversion(char *buf, size_t len) override;
    virtual esp_err_t get_interfaceversion(uint32_t *version) override;
    virtual esp_err_t get_name(char *buf, size_t len) override;
    virtual esp_err_t get_supportedactions(std::vector<std::string> &actions) override;

    virtual esp_err_t get_canreverse(bool *canreverse) override;
    virtual esp_err_t get_ismoving(bool *ismoving) override;
    virtual esp_err_t get_mechanicalposition(double *mechanicalposition) override;
    virtual esp_err_t get_position(double *position) override;
    virtual esp_err_t get_reverse(bool *reverse) override;
    virtual esp_err_t put_reverse(bool reverse) override;
    virtual esp_err_t get_stepsize(double *stepsize) override;
    virtual esp_err_t get_targetposition(double *targetposition) override;
    virtual esp_err_t put_halt() override;
    virtual esp_err_t put_move(double position) override;
    virtual esp_err_t put_moveabsolute(double position) override;
    virtual esp_err_t put_movemechanical(double position) override;
    virtual esp_err_t put_sync(double position) override;

  private:
    RotatorHW &_rotator;
    bool _connected;
};

#endif // ROTATOR_API_H
