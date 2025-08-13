#include "RotatorApi.h"

RotatorApi::RotatorApi(RotatorHW &rot) : Rotator(), _rotator(rot), _connected(false)
{
}

RotatorApi::~RotatorApi()
{
}

esp_err_t RotatorApi::action(const char *action, const char *parameters, char *buf, size_t len)
{
    return ALPACA_ERR_ACTION_NOT_IMPLEMENTED;
}

esp_err_t RotatorApi::commandblind(const char *command, bool raw)
{
    return ALPACA_ERR_ACTION_NOT_IMPLEMENTED;
}

esp_err_t RotatorApi::commandbool(const char *command, bool raw, bool *resp)
{
    return ALPACA_ERR_ACTION_NOT_IMPLEMENTED;
}

esp_err_t RotatorApi::commandstring(const char *action, bool raw, char *buf, size_t len)
{
    return ALPACA_ERR_ACTION_NOT_IMPLEMENTED;
}

esp_err_t RotatorApi::get_connected(bool *connected)
{
    *connected = _connected;
    return ALPACA_OK;
}

esp_err_t RotatorApi::set_connected(bool connected)
{
    _connected = connected;
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_description(char *buf, size_t len)
{
    strncpy(buf, "Camera Rotator Controller", len);
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_driverinfo(char *buf, size_t len)
{
    strncpy(buf, "AstroGeeks", len);
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_driverversion(char *buf, size_t len)
{
    strncpy(buf, ROTATOR_VERSION, len);
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_interfaceversion(uint32_t *version)
{
    *version = 2;
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_name(char *buf, size_t len)
{
    strncpy(buf, "AG2998", len);
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_supportedactions(std::vector<std::string> &actions)
{
    actions.clear();
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_canreverse(bool *canreverse)
{
    *canreverse = true;
    return ALPACA_OK;
}

esp_err_t RotatorApi::get_ismoving(bool *ismoving)
{
    *ismoving = _rotator.getIsMoving();
    return ALPACA_OK;
}
esp_err_t RotatorApi::get_mechanicalposition(double *mechanicalposition)
{
    *mechanicalposition = _rotator.getMechanicalPosition();
    return ALPACA_OK;
}
esp_err_t RotatorApi::get_position(double *position)
{
    *position = _rotator.getPosition();
    return ALPACA_OK;
}
esp_err_t RotatorApi::get_reverse(bool *reverse)
{
    *reverse = _rotator.getReverse();
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_reverse(bool reverse)
{
    _rotator.putReverse(reverse);
    return ALPACA_OK;
}
esp_err_t RotatorApi::get_stepsize(double *stepsize)
{
    *stepsize = 0.02;
    return ALPACA_OK;
}
esp_err_t RotatorApi::get_targetposition(double *targetposition)
{
    *targetposition = _rotator.getTargetPosition();
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_halt()
{
    _rotator.putHalt();
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_move(double position)
{
    _rotator.putRelativePosition(position);
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_moveabsolute(double position)
{
    _rotator.putAbsolutePosition(position);
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_movemechanical(double position)
{
    _rotator.putMechanicalPosition(position);
    return ALPACA_OK;
}
esp_err_t RotatorApi::put_sync(double position)
{
    _rotator.syncPosition(position);
    return ALPACA_OK;
}
