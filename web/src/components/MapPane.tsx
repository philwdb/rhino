import { useEffect, useLayoutEffect, useRef, useState } from 'react'
import type { MapInfo, POI, Pose } from '../types'

// Each map grid cell is displayed at this many CSS pixels.
const PX = 3

interface Transform { scale: number; tx: number; ty: number }

interface Props {
  mapInfo: MapInfo
  mapSeq: number
  pose: Pose | null
  path: [number, number][]
  pois: POI[]
  onNavigate: (x: number, y: number) => void
  onCreatePoi?: (label: string, x: number, y: number) => void
}

export default function MapPane({ mapInfo, mapSeq, pose, path, pois, onNavigate, onCreatePoi }: Props) {
  const vpRef = useRef<HTMLDivElement>(null)
  const [tr, setTr] = useState<Transform>({ scale: 1, tx: 0, ty: 0 })
  const [dragging, setDragging] = useState(false)
  const dragRef = useRef<{ mx: number; my: number; tx: number; ty: number } | null>(null)

  const imgW = mapInfo.width * PX
  const imgH = mapInfo.height * PX

  // Centre the map on first render
  useLayoutEffect(() => {
    const vp = vpRef.current
    if (!vp) return
    const { width, height } = vp.getBoundingClientRect()
    setTr({ scale: 1, tx: (width - imgW) / 2, ty: (height - imgH) / 2 })
  }, [imgW, imgH])

  // Mouse wheel zoom around cursor
  useEffect(() => {
    const vp = vpRef.current
    if (!vp) return
    function onWheel(e: WheelEvent) {
      e.preventDefault()
      const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15
      const rect = vp!.getBoundingClientRect()
      const cx = e.clientX - rect.left
      const cy = e.clientY - rect.top
      setTr(prev => ({
        scale: Math.max(0.3, Math.min(20, prev.scale * factor)),
        tx: cx - (cx - prev.tx) * factor,
        ty: cy - (cy - prev.ty) * factor,
      }))
    }
    vp.addEventListener('wheel', onWheel, { passive: false })
    return () => vp.removeEventListener('wheel', onWheel)
  }, [])

  function onMouseDown(e: React.MouseEvent) {
    if (e.button !== 0) return
    dragRef.current = { mx: e.clientX, my: e.clientY, tx: tr.tx, ty: tr.ty }
    setDragging(true)
  }

  function onMouseMove(e: React.MouseEvent) {
    if (!dragRef.current) return
    const dx = e.clientX - dragRef.current.mx
    const dy = e.clientY - dragRef.current.my
    setTr(prev => ({ ...prev, tx: dragRef.current!.tx + dx, ty: dragRef.current!.ty + dy }))
  }

  function onMouseUp(e: React.MouseEvent) {
    if (!dragRef.current) return
    const moved = Math.abs(e.clientX - dragRef.current.mx) + Math.abs(e.clientY - dragRef.current.my)
    dragRef.current = null
    setDragging(false)

    // Treat as click only if barely moved
    if (moved < 5) {
      const rect = vpRef.current!.getBoundingClientRect()
      const vx = e.clientX - rect.left
      const vy = e.clientY - rect.top
      const canvasX = (vx - tr.tx) / tr.scale
      const canvasY = (vy - tr.ty) / tr.scale
      const imgCol = canvasX / PX
      const imgRow = canvasY / PX
      const wx = mapInfo.origin_x + imgCol * mapInfo.resolution
      const wy = mapInfo.origin_y + (mapInfo.height - 1 - imgRow) * mapInfo.resolution

      if (onCreatePoi) {
        const label = prompt('POI label:')
        if (label?.trim()) onCreatePoi(label.trim(), wx, wy)
      } else {
        onNavigate(wx, wy)
      }
    }
  }

  // World → SVG pixel coords (SVG viewBox is in image pixels)
  function w2c(wx: number) { return (wx - mapInfo.origin_x) / mapInfo.resolution }
  function w2r(wy: number) { return mapInfo.height - 1 - (wy - mapInfo.origin_y) / mapInfo.resolution }

  const robotCol = pose ? w2c(pose.x) : null
  const robotRow = pose ? w2r(pose.y) : null
  const arrowLen = 6  // SVG units

  return (
    <div
      ref={vpRef}
      className={`map-viewport${dragging ? ' dragging' : ''}`}
      onMouseDown={onMouseDown}
      onMouseMove={onMouseMove}
      onMouseUp={onMouseUp}
      onMouseLeave={e => { if (dragging) onMouseUp(e) }}
    >
      <div
        className="map-canvas"
        style={{ transform: `translate(${tr.tx}px,${tr.ty}px) scale(${tr.scale})` }}
      >
        <img
          src={`/api/map/raw?t=${mapSeq}`}
          width={imgW}
          height={imgH}
          draggable={false}
          alt="occupancy map"
        />

        <svg
          viewBox={`0 0 ${mapInfo.width} ${mapInfo.height}`}
          width={imgW}
          height={imgH}
        >
          {/* A* path */}
          {path.length >= 2 && (
            <polyline
              points={path.map(([x, y]) => `${w2c(x)},${w2r(y)}`).join(' ')}
              stroke="#f97316"
              strokeWidth={0.6}
              fill="none"
              strokeLinejoin="round"
            />
          )}

          {/* POI markers */}
          {pois.map(p => (
            <g key={p.id} transform={`translate(${w2c(p.x)},${w2r(p.y)})`}>
              <circle r={3} fill="#60a5fa" stroke="#0a1a2a" strokeWidth={0.5} />
              <text
                y={-4}
                textAnchor="middle"
                fontSize={5}
                fill="#60a5fa"
                style={{ pointerEvents: 'none', fontFamily: 'system-ui' }}
              >
                {p.label}
              </text>
            </g>
          ))}

          {/* Robot */}
          {robotCol != null && robotRow != null && pose && (
            <g>
              <circle cx={robotCol} cy={robotRow} r={3} fill="#fbbf24" />
              <line
                x1={robotCol}
                y1={robotRow}
                x2={robotCol + arrowLen * Math.cos(pose.yaw)}
                y2={robotRow - arrowLen * Math.sin(pose.yaw)}
                stroke="white"
                strokeWidth={1}
                strokeLinecap="round"
              />
            </g>
          )}
        </svg>
      </div>

      <div className="map-hint">
        {onCreatePoi ? 'Click to place POI' : 'Click to navigate · scroll to zoom · drag to pan'}
      </div>
    </div>
  )
}
