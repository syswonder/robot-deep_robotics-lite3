/**
 * Keyboard Control Module for MuJoCo Scenes
 *
 * Provides configurable keyboard controls for different robots.
 * 统一使用异步 step() 模式，每个控制周期调用一次。
 */

// ============================================================================
// Keyboard Controller Class
// ============================================================================

export class KeyboardController {
  constructor() {
    this.enabled = false;
    this.config = null;
    this.model = null;
    this.data = null;
    this.mujoco = null;
    this.keyStates = {};
    this.customController = null;
    this.keyboardInputEnabled = false;

    // Bind event handlers
    this._onKeyDown = this._onKeyDown.bind(this);
    this._onKeyUp = this._onKeyUp.bind(this);
    this._onBlur = this._onBlur.bind(this);
  }

  /**
   * Get configuration for a robot
   * @param {object} robot - Normalized robot package metadata
   * @returns {object|null} - Configuration object or null if not configured
   */
  getConfig(robot) {
    return robot?.controller ?? null;
  }

  /**
   * Check if a robot has keyboard control configured
   * @param {object} robot - Normalized robot package metadata
   * @returns {boolean}
   */
  hasConfig(robot) {
    return Boolean(this.getConfig(robot));
  }

  /**
   * Enable keyboard control for a robot
   * @param {object} robot - Normalized robot package metadata
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module
   * @returns {Promise<boolean>} - Whether enabling was successful
   */
  async enable(robot, model, data, mujoco) {
    // Disable any existing control first
    this.disable();

    const config = this.getConfig(robot);
    if (!config) {
      console.log(`No keyboard control configured for robot: ${robot?.id ?? 'unknown'}`);
      return false;
    }

    this.config = config;
    this.model = model;
    this.data = data;
    this.mujoco = mujoco;

    // Create and initialize controller
    const moduleUrl = new URL(config.modulePath, document.baseURI).href;
    const controllerModule = await import(moduleUrl);
    const ControllerClass = controllerModule[config.export];
    if (typeof ControllerClass !== 'function') {
      throw new Error(`${robot.id}: controller export not found: ${config.export}`);
    }
    this.customController = new ControllerClass();
    await this.customController.initialize(model, data, mujoco);

    // Initialize key states from the package-local controller.
    this.keyStates = {};
    for (const key of this.customController.getControlKeys()) {
      this.keyStates[key] = false;
    }

    this.keyboardInputEnabled = new URLSearchParams(window.location.search).get('dev') === '1';
    if (this.keyboardInputEnabled) {
      document.addEventListener('keydown', this._onKeyDown);
      document.addEventListener('keyup', this._onKeyUp);
      window.addEventListener('blur', this._onBlur);
    }

    this.enabled = true;
    console.log(`Keyboard control enabled for robot: ${robot.id}`);
    return true;
  }

  /**
   * Disable keyboard control
   */
  disable() {
    if (!this.enabled) return;

    document.removeEventListener('keydown', this._onKeyDown);
    document.removeEventListener('keyup', this._onKeyUp);
    window.removeEventListener('blur', this._onBlur);

    this.enabled = false;
    this.config = null;
    this.model = null;
    this.data = null;
    this.mujoco = null;
    this.keyStates = {};
    this.customController = null;
    this.keyboardInputEnabled = false;
  }

  /**
   * 异步控制步进 - 每个控制周期调用一次
   * @returns {Promise<void>}
   */
  async step() {
    if (!this.enabled || !this.customController || !this.data) return;

    await this.customController.step(this.keyStates, this.model, this.data, this.mujoco);
  }

  /**
   * Get current control description for GUI display
   * @returns {string}
   */
  getDescription() {
    if (this.customController) {
      return this.customController.getDescription();
    }
    return this.config ? this.config.description : '';
  }

  setExternalControlEnabled(enabled) {
    this.customController?.setExternalControlEnabled?.(enabled);
  }

  setTwist(command) {
    return this.customController?.setTwist?.(command) ?? false;
  }

  setArmJointCommand(command) {
    return this.customController?.setArmJointCommand?.(command, this.model, this.data) ?? false;
  }

  setArmPoseCommand(command) {
    return this.customController?.setArmPoseCommand?.(command, this.model, this.data) ?? false;
  }

  startPickObject(name) {
    return this.customController?.startPickObject?.(name, this.model, this.data) ?? false;
  }

  getPickStatus() {
    return this.customController?.getPickStatus?.() ?? null;
  }

  emergencyStop() {
    return this.customController?.emergencyStop?.() ?? false;
  }

  resetRobot() {
    if (!this.customController || !this.model || !this.data) return false;
    this.customController.reset(this.model, this.data);
    return true;
  }

  getRobotState() {
    return this.customController?.getRobotState?.(this.model, this.data) ?? null;
  }

  _onKeyDown(event) {
    if (!this.enabled) return;

    // Ignore if typing in an input field
    if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
      return;
    }

    if (event.code in this.keyStates) {
      this.keyStates[event.code] = true;
      event.preventDefault();
    }
  }

  _onKeyUp(event) {
    if (!this.enabled) return;

    if (event.code in this.keyStates) {
      this.keyStates[event.code] = false;
      event.preventDefault();
    }
  }

  _onBlur() {
    if (!this.enabled) return;

    // Reset all key states when window loses focus
    // This prevents stuck keys when user clicks outside the window
    for (const key in this.keyStates) {
      this.keyStates[key] = false;
    }
  }
}

// Singleton instance for easy access
export const keyboardController = new KeyboardController();
