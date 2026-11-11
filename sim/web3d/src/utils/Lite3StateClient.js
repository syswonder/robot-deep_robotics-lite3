export class Lite3StateClient {
  constructor(demo) {
    this.demo = demo;
    this.endpoint = new URLSearchParams(window.location.search).get('stateUrl') || './api/state';
    this.status = { connected: false, frames: 0, lastError: '', lastFrameAgeMs: null };
    this.nextFetchMs = 0;
    this.pending = false;
    this.lastFrameWallMs = 0;
  }

  start() {
    this.demo.params.paused = true;
    this.update(0);
  }

  update(timeMs) {
    if (timeMs < this.nextFetchMs || this.pending) return;
    this.nextFetchMs = timeMs + 33;
    this.pending = true;
    fetch(this.endpoint, { cache: 'no-store' })
      .then((response) => {
        if (!response.ok) throw new Error(`state HTTP ${response.status}`);
        return response.json();
      })
      .then((state) => this._apply(state))
      .catch((error) => {
        this.status.connected = false;
        this.status.lastError = String(error);
        this._updatePanel();
      })
      .finally(() => { this.pending = false; });
  }

  _apply(state) {
    const qpos = Array.isArray(state.qpos) ? state.qpos : [];
    const qvel = Array.isArray(state.qvel) ? state.qvel : [];
    if (qpos.length !== this.demo.data.qpos.length || qvel.length !== this.demo.data.qvel.length) {
      throw new Error(`state/model mismatch qpos=${qpos.length}/${this.demo.data.qpos.length} qvel=${qvel.length}/${this.demo.data.qvel.length}`);
    }
    this.demo.data.qpos.set(qpos);
    this.demo.data.qvel.set(qvel);
    this.demo.data.time = Number(state.simulationTime || 0);
    this.demo.mujoco.mj_forward(this.demo.model, this.demo.data);
    this.status.connected = true;
    this.status.frames += 1;
    this.status.lastError = '';
    this.lastFrameWallMs = performance.now();
    this.status.lastFrameAgeMs = 0;
    this.latest = state;
    this._updatePanel();
  }

  async action(name) {
    const response = await fetch(`./api/${encodeURIComponent(name)}`, { method: 'POST' });
    const message = await response.text();
    if (!response.ok) throw new Error(message || `action HTTP ${response.status}`);
    return message;
  }

  _updatePanel() {
    const el = document.getElementById('lite3-status');
    if (!el) return;
    const s = this.latest;
    if (!this.status.connected || !s) {
      el.textContent = `Bridge disconnected${this.status.lastError ? ': ' + this.status.lastError : ''}`;
      return;
    }
    el.textContent = `House 187 · sim ${Number(s.simulationTime).toFixed(1)} s · ${s.paused ? 'paused' : 'running'} · ${s.mode}`;
  }
}
