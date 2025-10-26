/**
 * Operation Catalog Types
 * 操作カタログシステムの型定義
 */

// ============================================================
// 基本型
// ============================================================

export type Vector3 = [number, number, number];
export type Quaternion = [number, number, number, number];

export interface Pose3D {
  x: number;
  y: number;
  z: number;
  roll?: number;
  pitch?: number;
  yaw?: number;
  quaternion?: Quaternion;
}

export interface Range {
  min: number;
  max: number;
  unit: 'rad' | 'deg' | 'mm' | 'm';
}

// ============================================================
// 操作カタログエントリ
// ============================================================

export interface OperationCatalogEntry {
  // 識別情報
  id: string;
  type: ArticulatedType;
  location: string;
  
  // 操作仕様
  operation_spec: OperationSpec;
  
  // 状態検出
  state_detection?: StateDetection;
  
  // メタデータ
  meta: OperationMetadata;
  
  // タイムスタンプ
  created_at: string;
  updated_at: string;
  verified_at?: string;
}

export type ArticulatedType = 
  | 'door' 
  | 'drawer' 
  | 'lid' 
  | 'cabinet_door'
  | 'refrigerator'
  | 'dishwasher'
  | 'microwave'
  | 'oven'
  | 'window'
  | 'shutter'
  | 'custom';

// ============================================================
// 操作仕様
// ============================================================

export interface OperationSpec {
  // 事前動作（ノブ回転など）
  pre_actions?: PreAction[];
  
  // 主動作
  main_action: MainAction;
  
  // 事後動作（自動閉鎖の無効化など）
  post_actions?: PostAction[];
  
  // 機構情報
  mechanism: Mechanism;
  
  // 把持仕様
  grasp_specs: GraspSpec[];
  
  // 制約条件
  constraints?: OperationConstraints;
}

export interface PreAction {
  type: PreActionType;
  target: string;
  params: PreActionParams;
  duration_s?: number;
  required: boolean;
}

export type PreActionType = 
  | 'rotate_knob'
  | 'push_button'
  | 'lift_latch'
  | 'unlock'
  | 'release_catch'
  | 'wait';

export interface PreActionParams {
  axis?: Vector3;
  angle_deg?: number;
  distance_mm?: number;
  force_N?: number;
  hold_time_s?: number;
}

export interface MainAction {
  type: MainActionType;
  params: MainActionParams;
  fallback_strategy?: FallbackStrategy;
}

export type MainActionType = 
  | 'pull' 
  | 'push' 
  | 'slide_left'
  | 'slide_right'
  | 'slide_up'
  | 'slide_down'
  | 'lift'
  | 'lower'
  | 'rotate';

export interface MainActionParams {
  direction: Vector3;
  max_distance_mm?: number;
  max_angle_deg?: number;
  nominal_speed_mm_s: number;
  force_threshold_N: number;
  use_compliance: boolean;
}

export interface PostAction {
  type: PostActionType;
  params: Record<string, any>;
}

export type PostActionType = 
  | 'hold_open'
  | 'ensure_closed'
  | 'lock'
  | 'release';

// ============================================================
// 機構定義
// ============================================================

export interface Mechanism {
  joint_type: JointType;
  axis_origin: Vector3;
  axis_direction: Vector3;
  range: Range;
  rest_position: number;
  spring_loaded?: boolean;
  self_closing?: boolean;
  damping_coefficient?: number;
}

export type JointType = 
  | 'revolute'
  | 'prismatic'
  | 'fixed'
  | 'floating';

// ============================================================
// 把持仕様
// ============================================================

export interface GraspSpec {
  id: string;
  type: 'handle' | 'knob' | 'edge' | 'surface';
  pose: Pose3D;
  approach: Vector3;
  grasp_type: GraspType;
  opening_mm: number;
  priority: number;
  pre_grasp_offset_mm?: number;
  post_grasp_action?: string;
}

export type GraspType = 
  | 'cylindrical'
  | 'pinch'
  | 'lateral'
  | 'spherical'
  | 'hook'
  | 'power';

// ============================================================
// 制約条件
// ============================================================

export interface OperationConstraints {
  requires_clearance_mm?: number;
  max_opening_speed_mm_s?: number;
  max_closing_speed_mm_s?: number;
  collision_zones?: CollisionZone[];
  forbidden_angles_deg?: Range[];
  requires_human_assistance?: boolean;
}

export interface CollisionZone {
  id: string;
  description: string;
  geometry: 'box' | 'cylinder' | 'sphere';
  pose: Pose3D;
  dimensions: number[];
}

// ============================================================
// 状態検出
// ============================================================

export interface StateDetection {
  closed_indicators?: StateIndicator[];
  open_indicators?: StateIndicator[];
  intermediate_states?: IntermediateState[];
  sensor_checks?: SensorCheck[];
}

export interface StateIndicator {
  type: 'visual' | 'proximity' | 'contact' | 'acoustic';
  feature: string;
  confidence_weight: number;
}

export interface IntermediateState {
  name: string;
  position: number;
  indicators: StateIndicator[];
}

export interface SensorCheck {
  sensor_type: string;
  expected_value: any;
  tolerance: number;
}

// ============================================================
// フォールバック戦略
// ============================================================

export interface FallbackStrategy {
  conditions: FallbackCondition[];
  actions: FallbackAction[];
  max_attempts: number;
}

export interface FallbackCondition {
  type: 'force_exceeded' | 'no_movement' | 'timeout' | 'collision';
  threshold: number;
}

export interface FallbackAction {
  type: 'retry' | 'try_alternate_grasp' | 'increase_force' | 'abort';
  params?: Record<string, any>;
}

// ============================================================
// メタデータ
// ============================================================

export interface OperationMetadata {
  name: string;
  created_by: string;
  verified: boolean;
  confidence: ConfidenceLevel;
  common_names?: string[];
  brand_model?: string;
  version: string;
  success_rate?: number;
  avg_execution_time_s?: number;
  tags?: string[];
  notes?: string;
  related_items?: string[];
}

export type ConfidenceLevel = 
  | 'measured'    // 実測値
  | 'verified'    // 動作確認済み
  | 'estimated'   // 推定値
  | 'assumed';    // 仮定値

// ============================================================
// クエリ・レスポンス
// ============================================================

export interface OperationQuery {
  id?: string;
  type?: ArticulatedType;
  location?: string;
  tags?: string[];
  verified?: boolean;
  confidence_min?: ConfidenceLevel;
}

export interface OperationResponse {
  success: boolean;
  operation?: OperationCatalogEntry;
  alternatives?: OperationCatalogEntry[];
  warnings?: string[];
  errors?: string[];
}

// ============================================================
// 実行ステータス
// ============================================================

export interface OperationExecution {
  operation_id: string;
  status: ExecutionStatus;
  current_phase: ExecutionPhase;
  progress: number; // 0.0 - 1.0
  start_time: string;
  elapsed_time_s: number;
  estimated_remaining_s?: number;
  force_readings?: number[];
  position_readings?: number[];
  errors?: ExecutionError[];
}

export type ExecutionStatus = 
  | 'pending'
  | 'preparing'
  | 'executing'
  | 'paused'
  | 'completed'
  | 'failed'
  | 'aborted';

export type ExecutionPhase = 
  | 'pre_action'
  | 'main_action'
  | 'post_action'
  | 'verification';

export interface ExecutionError {
  code: string;
  message: string;
  phase: ExecutionPhase;
  timestamp: string;
  recoverable: boolean;
}

// ============================================================
// テンプレート
// ============================================================

export interface OperationTemplate {
  id: string;
  name: string;
  category: string;
  base_spec: Partial<OperationSpec>;
  variable_params: TemplateParam[];
  preview_image?: string;
  usage_count: number;
}

export interface TemplateParam {
  path: string; // JSONPath to parameter
  name: string;
  type: 'number' | 'string' | 'boolean' | 'vector3';
  default_value: any;
  options?: any[];
  validation?: ParamValidation;
}

export interface ParamValidation {
  min?: number;
  max?: number;
  pattern?: string;
  required?: boolean;
}
