export interface Pose {
  x: number
  y: number
  yaw: number
}

export interface RobotStatus {
  mode: string
  is_standing: boolean
  battery_pct: number
  vx: number
  vy: number
  omega: number
}

export interface NavStatus {
  goal: { x: number; y: number; yaw: number | null } | null
  exploring: boolean
  mode: 'astar' | 'direct'
}

export interface MapInfo {
  origin_x: number
  origin_y: number
  resolution: number
  width: number
  height: number
}

export interface POI {
  id: string
  label: string
  x: number
  y: number
  z: number
}
