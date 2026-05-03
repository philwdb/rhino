import { useEffect, useRef, useState } from 'react'

const CMD_GROUPS = [
  {
    label: 'Posture',
    cmds: [
      { label: 'Stand Up', cmd: 'StandUp' },
      { label: 'Balance', cmd: 'BalanceStand' },
      { label: 'Stand Down', cmd: 'StandDown' },
      { label: 'Sit', cmd: 'Sit' },
      { label: 'Recovery', cmd: 'RecoveryStand' },
    ],
  },
  {
    label: 'Moves',
    cmds: [
      { label: 'Hello', cmd: 'Hello' },
      { label: 'Stretch', cmd: 'Stretch' },
      { label: 'Wallow', cmd: 'Wallow' },
      { label: 'Wiggle Hips', cmd: 'WiggleHips' },
    ],
  },
  {
    label: 'Dance',
    cmds: [
      { label: 'Dance 1', cmd: 'Dance1' },
      { label: 'Dance 2', cmd: 'Dance2' },
      { label: 'Moonwalk', cmd: 'MoonWalk' },
      { label: 'Finger Heart', cmd: 'FingerHeart' },
    ],
  },
  {
    label: 'Stunts',
    cmds: [
      { label: 'Front Flip', cmd: 'FrontFlip' },
      { label: 'Back Flip', cmd: 'Backflip' },
      { label: 'Front Jump', cmd: 'FrontJump' },
      { label: 'Handstand', cmd: 'Handstand' },
    ],
  },
]

const VX = 0.4
const OMEGA = 0.8

export default function RobotPanel() {
  return (
    <div className="robot-panel">
      <div className="robot-panel-header">
        <span className="robot-panel-title">Go2 : Calf</span>
      </div>
      <div className="robot-panel-body">
        <CameraSection />
        <div className="panel-divider" />
        <TeleopSection />
        <div className="panel-divider" />
        <CommandsSection />
      </div>
    </div>
  )
}

function CameraSection() {
  const [error, setError] = useState(false)
  return error ? (
    <div className="camera-placeholder">No camera feed</div>
  ) : (
    <img
      className="camera-feed"
      src="/api/camera/stream"
      alt="camera"
      onError={() => setError(true)}
    />
  )
}

function TeleopSection() {
  const pressed = useRef(new Set<string>())
  const [keys, setKeys] = useState(new Set<string>())
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  function sendVel(p: Set<string>) {
    let vx = 0, omega = 0
    if (p.has('KeyW')) vx += VX
    if (p.has('KeyS')) vx -= VX * 0.6
    if (p.has('KeyA')) omega += OMEGA
    if (p.has('KeyD')) omega -= OMEGA
    fetch('/api/velocity', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vx, vy: 0, omega }),
    }).catch(() => {})
  }

  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      if (!['KeyW', 'KeyS', 'KeyA', 'KeyD'].includes(e.code)) return
      if (e.repeat) return
      e.preventDefault()
      pressed.current.add(e.code)
      setKeys(new Set(pressed.current))
      sendVel(pressed.current)
      if (!intervalRef.current) {
        intervalRef.current = setInterval(() => {
          if (pressed.current.size > 0) sendVel(pressed.current)
        }, 100)
      }
    }

    function onKeyUp(e: KeyboardEvent) {
      if (!['KeyW', 'KeyS', 'KeyA', 'KeyD', 'Space'].includes(e.code)) return
      e.preventDefault()
      if (e.code === 'Space') pressed.current.clear()
      else pressed.current.delete(e.code)
      setKeys(new Set(pressed.current))
      sendVel(pressed.current)
      if (pressed.current.size === 0 && intervalRef.current) {
        clearInterval(intervalRef.current)
        intervalRef.current = null
      }
    }

    function onBlur() {
      pressed.current.clear()
      setKeys(new Set())
      fetch('/api/velocity', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ vx: 0, vy: 0, omega: 0 }),
      }).catch(() => {})
      if (intervalRef.current) { clearInterval(intervalRef.current); intervalRef.current = null }
    }

    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', onBlur)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', onBlur)
      if (intervalRef.current) clearInterval(intervalRef.current)
    }
  }, [])

  const p = keys
  return (
    <div className="teleop-tab">
      <div className="dpad">
        <div className="dpad-empty" />
        <button className={p.has('KeyW') ? 'pressed' : ''}>↑</button>
        <div className="dpad-empty" />
        <button className={p.has('KeyA') ? 'pressed' : ''}>↺</button>
        <button onClick={() => {
          pressed.current.clear()
          setKeys(new Set())
          fetch('/api/stop', { method: 'POST' }).catch(() => {})
        }}>■</button>
        <button className={p.has('KeyD') ? 'pressed' : ''}>↻</button>
        <div className="dpad-empty" />
        <button className={p.has('KeyS') ? 'pressed' : ''}>↓</button>
        <div className="dpad-empty" />
      </div>
      <p className="teleop-hint">W/S forward/back · A/D rotate · Space stop</p>
    </div>
  )
}

function CommandsSection() {
  async function send(cmd: string) {
    await fetch(`/api/cmd/${cmd}`, { method: 'POST' })
  }

  return (
    <div className="commands-tab">
      {CMD_GROUPS.map(group => (
        <div key={group.label} className="cmd-group">
          <div className="cmd-group-label">{group.label}</div>
          <div className="cmd-grid">
            {group.cmds.map(({ label, cmd }) => (
              <button key={cmd} onClick={() => send(cmd)}>{label}</button>
            ))}
          </div>
        </div>
      ))}
    </div>
  )
}
