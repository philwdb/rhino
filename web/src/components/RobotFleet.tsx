import type { NavStatus, Pose, RobotStatus } from '../types'

export interface RobotDef {
  id: string
  name: string
}

// All currently known robots. Add entries here when the bimanual setup arrives.
export const ROBOTS: RobotDef[] = [
  { id: 'calf', name: 'Go2 : Calf' },
]

interface Props {
  connected: boolean
  status: RobotStatus | null
  pose: Pose | null
  navStatus: NavStatus | null
  selectedId: string | null
  onSelect: (id: string | null) => void
}

export default function RobotFleet({ connected, status, pose, navStatus, selectedId, onSelect }: Props) {
  return (
    <aside className="robot-fleet">
      <div className="fleet-header">
        <span className="fleet-title">Fleet</span>
        <span className="fleet-count">{ROBOTS.length}</span>
      </div>

      {ROBOTS.map(robot => {
        const selected = selectedId === robot.id
        const bat = status?.battery_pct ?? 0
        const batClass = bat < 15 ? 'critical' : bat < 30 ? 'low' : ''

        return (
          <div
            key={robot.id}
            className={`robot-card${selected ? ' selected' : ''}`}
            onClick={() => onSelect(selected ? null : robot.id)}
          >
            <div className="robot-card-header">
              <div className={`status-dot${connected ? ' connected' : ''}`} />
              <span className="robot-card-name">{robot.name}</span>
              <span className="robot-card-chevron">›</span>
            </div>

            <div className="battery-bar" style={{ marginBottom: 8 }}>
              <div className={`battery-fill ${batClass}`} style={{ width: `${bat}%` }} />
            </div>

            <div className="robot-card-stats">
              <div className="stat-row">
                <span>Battery</span>
                <span>{bat.toFixed(0)}%</span>
              </div>
              <div className="stat-row">
                <span>Mode</span>
                <span>{status?.mode ?? '—'}</span>
              </div>
              <div className="stat-row">
                <span>Standing</span>
                <span>{status?.is_standing ? 'yes' : 'no'}</span>
              </div>
              <div className="stat-row">
                <span>x</span>
                <span>{pose ? `${pose.x.toFixed(2)} m` : '—'}</span>
              </div>
              <div className="stat-row">
                <span>y</span>
                <span>{pose ? `${pose.y.toFixed(2)} m` : '—'}</span>
              </div>
              <div className="stat-row">
                <span>yaw</span>
                <span>{pose ? `${(pose.yaw * 180 / Math.PI).toFixed(1)}°` : '—'}</span>
              </div>
              <div className="stat-row">
                <span>Nav</span>
                <span>
                  {navStatus?.exploring
                    ? <span style={{ color: 'var(--orange)' }}>exploring</span>
                    : navStatus?.goal
                    ? <span style={{ color: 'var(--accent)' }}>navigating</span>
                    : 'idle'}
                </span>
              </div>
              <div className="stat-row">
                <span>Temp</span>
                <span>—</span>
              </div>
            </div>
          </div>
        )
      })}
    </aside>
  )
}
