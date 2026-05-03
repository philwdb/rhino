import { useState } from 'react'
import type { POI } from '../types'

interface Props {
  pois: POI[]
  onNavigateToPoi: (id: string) => void
  onDeletePoi: (id: string) => void
  onCreatePoi: (label: string) => void
}

export default function PlanSidebar({ pois, onNavigateToPoi, onDeletePoi, onCreatePoi }: Props) {
  const [label, setLabel] = useState('')

  function submit(e: React.FormEvent) {
    e.preventDefault()
    const trimmed = label.trim()
    if (!trimmed) return
    onCreatePoi(trimmed)
    setLabel('')
  }

  return (
    <aside className="sidebar">
      <div className="card-title" style={{ padding: '4px 0' }}>Points of Interest</div>

      <form className="poi-create-form" onSubmit={submit}>
        <input
          value={label}
          onChange={e => setLabel(e.target.value)}
          placeholder="New POI label…"
          autoFocus
        />
        <button type="submit">+</button>
      </form>

      <p style={{ fontSize: 11, color: 'var(--muted)', marginTop: 2 }}>
        Click the map to place a POI at that position.
      </p>

      <div className="poi-list" style={{ marginTop: 8 }}>
        {pois.length === 0 && (
          <div style={{ color: 'var(--muted)', fontSize: 12, padding: '4px 0' }}>
            No POIs saved yet.
          </div>
        )}
        {pois.map(p => (
          <div key={p.id} className="poi-item">
            <span className="poi-label" title={p.label}>{p.label}</span>
            <span className="poi-coords">{p.x.toFixed(1)}, {p.y.toFixed(1)}</span>
            <button onClick={() => onNavigateToPoi(p.id)} title="Navigate here">→</button>
            <button className="danger" onClick={() => onDeletePoi(p.id)} title="Delete">×</button>
          </div>
        ))}
      </div>
    </aside>
  )
}
