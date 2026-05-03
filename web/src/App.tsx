import { useEffect, useState } from 'react'
import type { MapInfo, NavStatus, POI, Pose, RobotStatus } from './types'
import Topbar from './components/Topbar'
import RobotFleet from './components/RobotFleet'
import PlanSidebar from './components/PlanSidebar'
import MapPane from './components/MapPane'
import RobotPanel from './components/RobotPanel'

export type Page = 'dashboard' | 'plan'

export default function App() {
  const [connected, setConnected] = useState(false)
  const [pose, setPose] = useState<Pose | null>(null)
  const [status, setStatus] = useState<RobotStatus | null>(null)
  const [path, setPath] = useState<[number, number][]>([])
  const [navStatus, setNavStatus] = useState<NavStatus | null>(null)
  const [mapInfo, setMapInfo] = useState<MapInfo | null>(null)
  const [pois, setPois] = useState<POI[]>([])
  const [mapSeq, setMapSeq] = useState(0)
  const [page, setPage] = useState<Page>('dashboard')
  const [selectedRobotId, setSelectedRobotId] = useState<string | null>(null)

  // Poll state + nav every 300 ms
  useEffect(() => {
    async function poll() {
      try {
        const [sRes, nRes] = await Promise.all([
          fetch('/api/state'),
          fetch('/api/navigate/status'),
        ])
        const s = await sRes.json()
        const n = await nRes.json()
        setPose(s.pose ?? null)
        setStatus(s.status)
        setPath(s.path ?? [])
        setNavStatus(n)
        setConnected(true)
      } catch {
        setConnected(false)
      }
    }
    poll()
    const id = setInterval(poll, 300)
    return () => clearInterval(id)
  }, [])

  // Tick map refresh at 500 ms
  useEffect(() => {
    const id = setInterval(() => setMapSeq(s => s + 1), 500)
    return () => clearInterval(id)
  }, [])

  // Map info once
  useEffect(() => {
    fetch('/api/map/info').then(r => r.json()).then(setMapInfo).catch(() => {})
  }, [])

  // POIs every 3 s
  useEffect(() => {
    function load() {
      fetch('/api/pois').then(r => r.json()).then(setPois).catch(() => {})
    }
    load()
    const id = setInterval(load, 3000)
    return () => clearInterval(id)
  }, [])

  async function navigate(x: number, y: number) {
    await fetch('/api/navigate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y }),
    })
  }

  async function stop() {
    await Promise.all([
      fetch('/api/stop', { method: 'POST' }),
      fetch('/api/navigate/cancel', { method: 'POST' }),
    ])
  }

  async function toggleExplore() {
    if (!navStatus) return
    await fetch(navStatus.exploring ? '/api/explore/stop' : '/api/explore/start', { method: 'POST' })
  }

  async function setMode(mode: 'astar' | 'direct') {
    await fetch('/api/navigate/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    })
  }

  async function createPoi(label: string, x?: number, y?: number) {
    await fetch('/api/pois', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ label, x, y }),
    })
    const updated = await fetch('/api/pois').then(r => r.json())
    setPois(updated)
  }

  async function deletePoi(id: string) {
    await fetch(`/api/pois/${id}`, { method: 'DELETE' })
    setPois(prev => prev.filter(p => p.id !== id))
  }

  async function navigateToPoi(id: string) {
    await fetch(`/api/pois/${id}/navigate`, { method: 'POST' })
  }

  // Close robot panel when switching to Plan
  function handlePage(p: Page) {
    setPage(p)
    if (p === 'plan') setSelectedRobotId(null)
  }

  return (
    <div className="app">
      <Topbar
        connected={connected}
        page={page}
        onPage={handlePage}
        navStatus={navStatus}
        onStop={stop}
        onToggleExplore={toggleExplore}
        onSetMode={setMode}
      />

      {page === 'dashboard' && (
        <div className="layout">
          <RobotFleet
            connected={connected}
            status={status}
            pose={pose}
            navStatus={navStatus}
            selectedId={selectedRobotId}
            onSelect={setSelectedRobotId}
          />
          <main className="main">
            {mapInfo && (
              <MapPane
                mapInfo={mapInfo}
                mapSeq={mapSeq}
                pose={pose}
                path={path}
                pois={pois}
                onNavigate={navigate}
              />
            )}
          </main>
          {selectedRobotId && <RobotPanel />}
        </div>
      )}

      {page === 'plan' && (
        <div className="layout">
          <PlanSidebar
            pois={pois}
            onNavigateToPoi={navigateToPoi}
            onDeletePoi={deletePoi}
            onCreatePoi={label => createPoi(label)}
          />
          <main className="main">
            {mapInfo && (
              <MapPane
                mapInfo={mapInfo}
                mapSeq={mapSeq}
                pose={pose}
                path={path}
                pois={pois}
                onNavigate={navigate}
                onCreatePoi={createPoi}
              />
            )}
          </main>
        </div>
      )}
    </div>
  )
}
