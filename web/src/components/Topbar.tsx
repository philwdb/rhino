import type { NavStatus } from '../types'
import type { Page } from '../App'

interface Props {
  connected: boolean
  page: Page
  onPage: (p: Page) => void
  navStatus: NavStatus | null
  onStop: () => void
  onToggleExplore: () => void
  onSetMode: (m: 'astar' | 'direct') => void
}

export default function Topbar({ connected, page, onPage, navStatus, onStop, onToggleExplore, onSetMode }: Props) {
  const exploring = navStatus?.exploring ?? false
  const mode = navStatus?.mode ?? 'astar'
  const hasGoal = navStatus?.goal != null

  return (
    <header className="topbar">
      <span className="topbar-brand">rhino</span>
      <div className={`topbar-dot${connected ? ' connected' : ''}`} title={connected ? 'connected' : 'disconnected'} />

      <nav className="topbar-nav">
        <button className={page === 'dashboard' ? 'active' : ''} onClick={() => onPage('dashboard')}>
          Dashboard
        </button>
        <button className={page === 'plan' ? 'active' : ''} onClick={() => onPage('plan')}>
          Plan
        </button>
      </nav>

      <div className="topbar-spacer" />

      <div className="topbar-actions">
        {exploring && <span className="badge badge-orange">exploring</span>}
        {hasGoal && !exploring && <span className="badge badge-blue">navigating</span>}

        <button
          className={exploring ? 'active' : ''}
          onClick={onToggleExplore}
          title="Toggle autonomous exploration"
        >
          {exploring ? '⏹ Stop explore' : '⟳ Explore'}
        </button>

        <button
          className={mode === 'direct' ? 'active' : ''}
          onClick={() => onSetMode(mode === 'astar' ? 'direct' : 'astar')}
          title="Toggle plan mode"
        >
          {mode === 'astar' ? 'A*' : 'Direct'}
        </button>

        <button className="danger" onClick={onStop}>■ Stop</button>
      </div>
    </header>
  )
}
