'use strict';

// The controller is independent of ROSLIB/DOM so failure paths can be tested.
class LineCalibration {
  constructor(request, changed, wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms))) {
    this.request = request;
    this.changed = changed;
    this.wait = wait;
    this.connected = false;
    this.status = null;
    this.statusAt = 0;
    this.active = false;
    this.busy = false;
    this.step = 'white';
    this.message = '';
    this.generation = 0;
  }

  connection(connected) {
    this.connected = connected;
    this.status = null;
    this.statusAt = 0;
    this.cancel(connected ? '' : 'Not connected. The saved calibration is kept.');
  }

  update(status) {
    this.status = status;
    this.statusAt = Date.now();
    this.changed();
  }

  available() {
    return this.connected && this.status?.supported === true && Date.now() - this.statusAt < 2000;
  }

  cancel(message = 'Calibration closed. The saved settings are kept.') {
    this.generation++;
    this.active = false;
    this.busy = false;
    this.message = message;
    this.changed();
  }

  modeChanged(mode) {
    if (this.active && this.step !== 'starting' && mode !== 'idle') {
      this.cancel('The robot mode changed. Restart calibration in Idle mode.');
    }
  }

  async start() {
    if (!this.available() || this.active || this.busy) return;
    const generation = ++this.generation;
    this.active = true;
    this.busy = true;
    this.step = 'starting';
    this.message = 'Switching the robot to Idle mode…';
    this.changed();
    try {
      const result = await this.request('idle');
      if (generation !== this.generation) return;
      if (!result.success) throw new Error(result.message);
      this.step = 'white';
      this.busy = false;
      this.message = '1/2 · Place a white surface under all eight channels. Keep the height constant.';
      this.changed();
    } catch (error) {
      if (generation === this.generation) this.cancel(error.message);
    }
  }

  async capture() {
    if (!this.active || this.busy || !this.available() || !this.status.ready) return;
    const generation = this.generation;
    const surface = this.step;
    if (surface !== 'white' && surface !== 'black') return;
    this.busy = true;
    this.message = 'Measuring… Keep the surface and the sensor still.';
    this.changed();
    try {
      // Fill the server's one-second window AFTER the user's explicit click.
      await this.wait(1200);
      if (generation !== this.generation) return;
      if (!this.available() || !this.status.ready) throw new Error('No live readings; check the connection and Idle mode.');
      const result = await this.request(surface);
      if (generation !== this.generation) return;
      if (!result.success) throw new Error(result.message);
      this.busy = false;
      if (surface === 'white') {
        this.step = 'black';
        this.message = '2/2 · Replace the white with a matte black surface covering all eight channels. Do not change the height.';
      } else {
        this.active = false;
        this.message = result.message;
      }
      this.changed();
    } catch (error) {
      if (generation !== this.generation) return;
      this.busy = false;
      this.message = `${error.message} Check the surface and the saved references, then measure again.`;
      this.changed();
    }
  }
}

if (typeof module !== 'undefined') module.exports = { LineCalibration };
