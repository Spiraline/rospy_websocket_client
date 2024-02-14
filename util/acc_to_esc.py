class ACC2ESC():
  def __init__(self):
    self._curr_ax = 0.0
    self._accel = 0.0
    self._brake = 0.0

    self._discrete_time_integrater = 0.0

  def axCallback(self, desired_ax):
    # Use when stop_req_u1 = true
    # cliped_desried_ax = min(-0.5, desired_ax)

    ax_diff = desired_ax - self._curr_ax

    self._accel = 0.05 * ax_diff + self._discrete_time_integrater
    
    # Saturation
    if self._accel > 1.0: self._accel = 1.0
    elif self._accel < -1.0: self._accel = -1.0

    if self._accel >= 0.0:
      self._brake = 0.0
    else:
      self._brake = -self._accel
      self._accel = 0.0
    
    self._discrete_time_integrater += 0.001 * ax_diff * 0.001
    if self._discrete_time_integrater >= 1.0: self._discrete_time_integrater = 1.0
    elif self._discrete_time_integrater < -1.0: self._discrete_time_integrater = -1.0
  
  def setCurrAx(self, ax):
    self._curr_ax = ax

  def getcurrAx(self):
    return self._curr_ax
  
  def getAccel(self):
    return self._accel
  
  def getBrake(self):
    return self._brake