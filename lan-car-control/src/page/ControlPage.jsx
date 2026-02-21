import { useEffect, useRef, useState } from 'react'
import Radar from '../components/Radar'
import InfraredSensor from '../components/InfraredSensor'
import './ControlPage.css'

const clamp = (value, min, max) => Math.max(min, Math.min(max, value))

const MAX_LINEAR = 0.6
const MAX_LATERAL = 0.6
const MAX_ANGULAR = 1.2
const SERVO_RANGE = 60

const buildTwist = (linearX, linearY, angularZ) => ({
  linear: { x: linearX, y: linearY, z: 0 },
  angular: { x: 0, y: 0, z: angularZ },
})

function Joystick({ label, axis = 'both', size = 160, onChange }) {
  const baseRef = useRef(null)
  const [position, setPosition] = useState({ x: 0, y: 0 })
  const draggingRef = useRef(false)

  const updatePosition = (event) => {
    if (!baseRef.current) {
      return
    }
    const rect = baseRef.current.getBoundingClientRect()
    const radius = rect.width / 2
    const dx = event.clientX - rect.left - radius
    const dy = event.clientY - rect.top - radius
    let nx = dx / radius
    let ny = dy / radius

    if (axis === 'x') {
      ny = 0
      nx = clamp(nx, -1, 1)
    } else if (axis === 'y') {
      nx = 0
      ny = clamp(ny, -1, 1)
    } else {
      const length = Math.hypot(nx, ny)
      if (length > 1) {
        nx /= length
        ny /= length
      }
    }

    setPosition({ x: nx, y: ny })
    if (onChange) {
      onChange(nx, ny)
    }
  }

  const handleDown = (event) => {
    draggingRef.current = true
    event.currentTarget.setPointerCapture(event.pointerId)
    updatePosition(event)
  }

  const handleMove = (event) => {
    if (!draggingRef.current) {
      return
    }
    updatePosition(event)
  }

  const handleUp = (event) => {
    if (!draggingRef.current) {
      return
    }
    draggingRef.current = false
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId)
    }
    setPosition({ x: 0, y: 0 })
    if (onChange) {
      onChange(0, 0)
    }
  }

  const radius = size / 2
  const handleStyle = {
    transform: `translate(${position.x * radius}px, ${position.y * radius}px)`,
  }

  return (
    <div className="joystick-wrapper">
      <div className="joystick-title">{label}</div>
      <div
        className="joystick"
        ref={baseRef}
        style={{ width: size, height: size }}
        onPointerDown={handleDown}
        onPointerMove={handleMove}
        onPointerUp={handleUp}
        onPointerCancel={handleUp}
      >
        <div className="joystick-center" />
        <div className="joystick-handle" style={handleStyle} />
      </div>
    </div>
  )
}

function ControlPage() {
  const [wsHost, setWsHost] = useState('')
  const [connection, setConnection] = useState('disconnected')
  const [statusText, setStatusText] = useState('未连接')
  const [linearX, setLinearX] = useState(0)
  const [linearY, setLinearY] = useState(0)
  const [angularZ, setAngularZ] = useState(0)
  const [servoPan, setServoPan] = useState(90)
  const [servoTilt, setServoTilt] = useState(90)
  const [cameraOn, setCameraOn] = useState(false)
  const [imageSrc, setImageSrc] = useState('')
  const [hasRawFrame, setHasRawFrame] = useState(false)
  const [imageTopic, setImageTopic] = useState('/image_raw')
  const [cameraTopics, setCameraTopics] = useState([])
  const [cameraTypes, setCameraTypes] = useState({})
  const [cameraStats, setCameraStats] = useState({
    lastTopic: '',
    lastAt: null,
    width: null,
    height: null,
    encoding: '',
    format: '',
    dataSize: null,
    receiveCount: 0,
  })
  const [wsStats, setWsStats] = useState({
    lastOp: '',
    lastTopic: '',
    lastAt: null,
    totalMessages: 0,
    rawSize: null,
  })
  const [wsPayloadPreview, setWsPayloadPreview] = useState('')
  const [wsSendPreview, setWsSendPreview] = useState('')
  const [ultrasonicRange, setUltrasonicRange] = useState(null)
  const [infraredValue, setInfraredValue] = useState(0)
  const wsRef = useRef(null)
  const canvasRef = useRef(null)
  const chassisSendAtRef = useRef(0)
  const servoSendAtRef = useRef(0)
  const cameraTopicRef = useRef('/image_raw')
  const cameraTopicsRef = useRef(new Set(['/image_raw']))
  const fragmentStoreRef = useRef(new Map())

  const isConnected = connection === 'connected'

  const sendMessage = (payload) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      setStatusText('未连接，无法发送')
      return
    }
    const payloadText = JSON.stringify(payload)
    setWsSendPreview(payloadText.length > 1200 ? `${payloadText.slice(0, 1200)}…` : payloadText)
    wsRef.current.send(JSON.stringify(payload))
  }

  const publish = (topic, msg, type) => {
    sendMessage({
      op: 'publish',
      topic,
      type,
      msg,
    })
  }

  const socketReady = () =>
    wsRef.current && wsRef.current.readyState === WebSocket.OPEN

  const publishChassis = (x, y, z) => {
    publish('/cmd_vel', buildTwist(x, y, z), 'geometry_msgs/Twist')
  }

  const publishServo = (pan, tilt) => {
    publish('/servo_angle', { data: pan }, 'std_msgs/Int32')
    publish('/servo_angle_tilt', { data: tilt }, 'std_msgs/Int32')
  }

  const sendChassisThrottled = (x, y, z) => {
    if (!socketReady()) {
      return
    }
    const now = Date.now()
    if (now - chassisSendAtRef.current < 60) {
      return
    }
    chassisSendAtRef.current = now
    publishChassis(x, y, z)
  }

  const sendServoThrottled = (pan, tilt) => {
    if (!socketReady()) {
      return
    }
    const now = Date.now()
    if (now - servoSendAtRef.current < 80) {
      return
    }
    servoSendAtRef.current = now
    publishServo(pan, tilt)
  }

  const toBase64 = (data) => {
    if (typeof data === 'string') {
      return data
    }
    let view = null
    if (data instanceof ArrayBuffer) {
      view = new Uint8Array(data)
    } else if (ArrayBuffer.isView(data)) {
      view = new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
    } else if (Array.isArray(data)) {
      view = Uint8Array.from(data)
    }
    if (!view || view.length === 0) {
      return ''
    }
    let binary = ''
    const chunkSize = 8192
    for (let i = 0; i < view.length; i += chunkSize) {
      binary += String.fromCharCode(...view.subarray(i, i + chunkSize))
    }
    return btoa(binary)
  }

  const resolveImageSrc = (msg) => {
    if (!msg || !msg.data) {
      return ''
    }
    const base64 = toBase64(msg.data)
    if (!base64) {
      return ''
    }
    const format = String(msg.format || 'jpeg').toLowerCase()
    const mime = format.includes('png') ? 'image/png' : 'image/jpeg'
    return `data:${mime};base64,${base64}`
  }

  const toByteArray = (data) => {
    if (!data) {
      return null
    }
    if (data instanceof Uint8Array) {
      return data
    }
    if (data instanceof ArrayBuffer) {
      return new Uint8Array(data)
    }
    if (ArrayBuffer.isView(data)) {
      return new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
    }
    if (Array.isArray(data)) {
      return Uint8Array.from(data)
    }
    if (typeof data === 'string') {
      const binary = atob(data)
      const bytes = new Uint8Array(binary.length)
      for (let i = 0; i < binary.length; i += 1) {
        bytes[i] = binary.charCodeAt(i)
      }
      return bytes
    }
    return null
  }

  const renderRawImage = (msg) => {
    if (!canvasRef.current || !msg) {
      return false
    }
    const width = Number(msg.width)
    const height = Number(msg.height)
    if (!width || !height) {
      return false
    }
    const encoding = String(msg.encoding || '').toLowerCase()
    const data = toByteArray(msg.data)
    if (!data || data.length === 0) {
      return false
    }
    const canvas = canvasRef.current
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width
      canvas.height = height
    }
    const ctx = canvas.getContext('2d')
    if (!ctx) {
      return false
    }
    const imageData = ctx.createImageData(width, height)
    const output = imageData.data
    if (encoding === 'rgb8' || encoding === 'bgr8') {
      for (let i = 0, j = 0; i < data.length && j < output.length; i += 3, j += 4) {
        const r = encoding === 'rgb8' ? data[i] : data[i + 2]
        const g = data[i + 1]
        const b = encoding === 'rgb8' ? data[i + 2] : data[i]
        output[j] = r
        output[j + 1] = g
        output[j + 2] = b
        output[j + 3] = 255
      }
    } else if (encoding === 'rgba8' || encoding === 'bgra8') {
      for (let i = 0, j = 0; i < data.length && j < output.length; i += 4, j += 4) {
        const r = encoding === 'rgba8' ? data[i] : data[i + 2]
        const g = data[i + 1]
        const b = encoding === 'rgba8' ? data[i + 2] : data[i]
        const a = data[i + 3]
        output[j] = r
        output[j + 1] = g
        output[j + 2] = b
        output[j + 3] = a
      }
    } else if (encoding === 'mono8') {
      for (let i = 0, j = 0; i < data.length && j < output.length; i += 1, j += 4) {
        const v = data[i]
        output[j] = v
        output[j + 1] = v
        output[j + 2] = v
        output[j + 3] = 255
      }
    } else if (
      encoding.includes('yuyv') ||
      encoding.includes('yuy2') ||
      encoding.includes('yuv422')
    ) {
      const clampValue = (value) => Math.max(0, Math.min(255, value))
      for (let i = 0, j = 0; i + 3 < data.length && j + 7 < output.length; i += 4, j += 8) {
        const y0 = data[i]
        const u = data[i + 1]
        const y1 = data[i + 2]
        const v = data[i + 3]
        const c0 = y0 - 16
        const c1 = y1 - 16
        const d = u - 128
        const e = v - 128
        const r0 = clampValue((298 * c0 + 409 * e + 128) >> 8)
        const g0 = clampValue((298 * c0 - 100 * d - 208 * e + 128) >> 8)
        const b0 = clampValue((298 * c0 + 516 * d + 128) >> 8)
        const r1 = clampValue((298 * c1 + 409 * e + 128) >> 8)
        const g1 = clampValue((298 * c1 - 100 * d - 208 * e + 128) >> 8)
        const b1 = clampValue((298 * c1 + 516 * d + 128) >> 8)
        output[j] = r0
        output[j + 1] = g0
        output[j + 2] = b0
        output[j + 3] = 255
        output[j + 4] = r1
        output[j + 5] = g1
        output[j + 6] = b1
        output[j + 7] = 255
      }
    } else {
      return false
    }
    ctx.putImageData(imageData, 0, 0)
    return true
  }

  const renderCameraMessage = (msg, topic) => {
    if (!msg) {
      return false
    }
    const dataSize = Array.isArray(msg.data)
      ? msg.data.length
      : typeof msg.data === 'string'
        ? msg.data.length
        : msg.data?.byteLength || null
    setCameraStats((prev) => ({
      ...prev,
      lastTopic: topic || cameraTopicRef.current,
      lastAt: Date.now(),
      width: msg.width ?? prev.width,
      height: msg.height ?? prev.height,
      encoding: msg.encoding ?? prev.encoding,
      format: msg.format ?? prev.format,
      dataSize,
      receiveCount: prev.receiveCount + 1,
    }))
    if (msg.format || (!msg.height && !msg.width)) {
      const src = resolveImageSrc(msg)
      if (src) {
        setImageSrc(src)
        setHasRawFrame(false)
        return true
      }
    } else if (msg.header && (msg.format === undefined)) {
        // Fallback for compressed image if format is missing but looks like compressed
        // Some implementations might not send 'format' field correctly or it's 'jpeg'
        const src = resolveImageSrc({...msg, format: 'jpeg'})
        if (src) {
            setImageSrc(src)
            setHasRawFrame(false)
            return true
        }
    }
    if (msg.height && msg.width && msg.encoding) {
      const rendered = renderRawImage(msg)
      if (rendered) {
        setImageSrc('')
        setHasRawFrame(true)
      }
      return rendered
    }
    return false
  }

  const normalizeTopic = (value) =>
    String(value || '').trim().replace(/\/+$/, '')

  const withLeadingSlash = (value) =>
    value.startsWith('/') ? value : `/${value}`

  const isCameraTopicMatch = (value) => {
    const normalized = normalizeTopic(value)
    if (!normalized) {
      return false
    }
    const withSlash = withLeadingSlash(normalized)
    const withoutSlash = withSlash.slice(1)
    return (
      cameraTopicsRef.current.has(withSlash) ||
      cameraTopicsRef.current.has(withoutSlash) ||
      cameraTopicsRef.current.has(normalized)
    )
  }

  const handleRosMessage = (payload) => {
    const op = payload.op || payload.operation
    const msg = payload.msg || payload.message
    if ((op === 'publish' || op === 'message') && isCameraTopicMatch(payload.topic) && msg) {
      renderCameraMessage(msg, payload.topic)
    }
    
    if ((op === 'publish' || op === 'message') && withLeadingSlash(payload.topic) === '/ultrasonic/range' && msg) {
      setUltrasonicRange(msg.range)
    }

    if ((op === 'publish' || op === 'message') && withLeadingSlash(payload.topic) === '/line_sensor' && msg) {
      setInfraredValue(msg.data)
    }
  }

  const handleFragment = (payload) => {
    const id = payload.id || payload.fragment_id || payload.fragmentId
    if (!id) {
      return
    }
    const decodeFragment = (data) => {
      // 临时修复：rosbridge 发送的 JSON 字符串形式的分片已经是解码好的内容
      // 不需要二次 TextDecoder，也不需要 fromCharCode
      if (typeof data === 'string') {
        return data
      }
      if (data instanceof ArrayBuffer) {
        return new TextDecoder().decode(new Uint8Array(data))
      }
      if (ArrayBuffer.isView(data)) {
        return new TextDecoder().decode(
          new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
        )
      }
      if (Array.isArray(data)) {
        return String.fromCharCode(...data)
      }
      return ''
    }
    const total = Number(payload.total || payload.total_parts || payload.totalParts)
    const num = Number(payload.num || payload.index || payload.part)
    if (!Number.isFinite(total) || total <= 0 || !Number.isFinite(num)) {
      return
    }
    let index = num
    if (num >= total && num - 1 < total) {
      index = num - 1
    }
    if (index < 0 || index >= total) {
      return
    }
    const store = fragmentStoreRef.current
    let entry = store.get(id)
    if (!entry || entry.total !== total) {
      entry = { total, parts: Array(total).fill(''), received: 0 }
      store.set(id, entry)
    }
    if (!entry.parts[index]) {
      entry.parts[index] = decodeFragment(payload.data)
      entry.received += 1
    }
    if (entry.received >= entry.total) {
      store.delete(id)
      const joined = entry.parts.join('')
      try {
        // rosbridge 发送的分片 data 实际上是普通的字符串（JSON片段）
        // 不需要 Base64 解码，直接拼接即可
        const reconstructed = JSON.parse(joined)
        handleRosMessage(reconstructed)
      } catch (e) {
        console.error('分片重组失败', e)
        setStatusText('分片解析失败')
      }
    }
  }

  const buildCameraTopics = (topic) => {
    const normalized = normalizeTopic(topic)
    if (!normalized) {
      return []
    }
    const withSlash = withLeadingSlash(normalized)
    const topics = new Set([withSlash])
    if (withSlash.endsWith('/compressed')) {
      const base = withSlash.slice(0, -'/compressed'.length) || '/'
      topics.add(base)
    } else {
      topics.add(`${withSlash}/compressed`)
    }
    return Array.from(topics).filter(Boolean)
  }

  const subscribeCamera = (topic) => {
    const topics = buildCameraTopics(topic)
    // 恢复默认订阅所有可能的话题，以防 compressed 不可用
    topics.push('/image_raw', '/image_raw/compressed')
    
    cameraTopicsRef.current = new Set(topics)
    topics.forEach((item) => {
      // 明确类型为 CompressedImage
      const type = item.endsWith('compressed') ? 'sensor_msgs/CompressedImage' : 'sensor_msgs/Image'
      sendMessage({
        op: 'subscribe',
        topic: item,
        type,
      })
    })
  }

  const unsubscribeCamera = (topic) => {
    const topics = buildCameraTopics(topic)
    topics.push('/image_raw', '/image_raw/compressed')
    topics.forEach((item) => {
      sendMessage({
        op: 'unsubscribe',
        topic: item,
      })
    })
    cameraTopicsRef.current = new Set()
  }

  const connect = () => {
    if (wsRef.current) {
      wsRef.current.close()
    }
    setConnection('connecting')
    setStatusText('连接中...')
    const socket = new WebSocket(`ws://${wsHost}:9090`)
    // socket.binaryType = 'arraybuffer'
    wsRef.current = socket

    socket.onopen = () => {
      setConnection('connected')
      setStatusText('已连接')

      // Subscribe to ultrasonic
      sendMessage({
        op: 'subscribe',
        topic: '/ultrasonic/range',
        type: 'sensor_msgs/Range',
      })

      // Subscribe to line sensor
      sendMessage({
        op: 'subscribe',
        topic: '/line_sensor',
        type: 'std_msgs/UInt8',
      })

      if (cameraOn) {
        cameraTopicRef.current = imageTopic
        subscribeCamera(imageTopic)
      }
    }

    socket.onclose = () => {
      setConnection('disconnected')
      setStatusText('已断开')
      setImageSrc('')
      setHasRawFrame(false)
      setCameraTopics([])
      setCameraTypes({})
    }

    socket.onerror = () => {
      setConnection('disconnected')
      setStatusText('连接失败')
    }

    socket.onmessage = async (event) => {
      try {
        let raw = event.data
        let rawSize = null
        if (raw instanceof Blob) {
          rawSize = raw.size
          raw = await raw.text()
        } else if (raw instanceof ArrayBuffer) {
          rawSize = raw.byteLength
          raw = new TextDecoder().decode(new Uint8Array(raw))
        } else if (typeof raw === 'string') {
          rawSize = raw.length
        }
        if (typeof raw !== 'string') {
          return
        }

        // 临时调试：如果包含 "publish" 或 "image_raw" 直接显示前 200 字符
        if (raw.includes('image_raw') || raw.includes('publish')) {
          setWsPayloadPreview(() => {
            if (raw.length > 500) {
              return `[RAW] ${raw.slice(0, 500)}…`
            }
            return `[RAW] ${raw}`
          })
        }

        const data = JSON.parse(raw)
        setWsPayloadPreview(() => {
          const text = JSON.stringify(data)
          if (text.length > 1200) {
            return `${text.slice(0, 1200)}…`
          }
          return text
        })
        const op = data.op || data.operation
        setWsStats((prev) => ({
          ...prev,
          lastOp: op || prev.lastOp,
          lastTopic: data.topic || prev.lastTopic,
          lastAt: Date.now(),
          totalMessages: prev.totalMessages + 1,
          rawSize,
        }))
        if (op === 'service_response') {
          const result = data.values || {}
          const topics = Array.isArray(result.topics) ? result.topics : []
          const types = Array.isArray(result.types) ? result.types : []
          const mapping = {}
          topics.forEach((topic, index) => {
            mapping[topic] = types[index] || ''
          })
          setCameraTopics(topics)
          setCameraTypes(mapping)
          return
        }
        if (op === 'fragment') {
          // 调试：打印第一个分片
          if ((data.num === 0 || data.num === 1) && !data.data.startsWith('{')) {
            setWsPayloadPreview(() => `[FRAGMENT 0] ${data.data.slice(0, 100)}...`)
          }
          handleFragment(data)
          return
        }
        handleRosMessage(data)
      } catch {
        setStatusText('接收数据异常')
      }
    }
  }

  const disconnect = () => {
    if (wsRef.current) {
      wsRef.current.close()
    }
  }

  const sendChassis = () => {
    publishChassis(linearX, linearY, angularZ)
    setStatusText('已发送底盘指令')
  }

  const stopChassis = () => {
    setLinearX(0)
    setLinearY(0)
    setAngularZ(0)
    publishChassis(0, 0, 0)
    setStatusText('已停止底盘')
  }

  const sendServo = () => {
    publishServo(servoPan, servoTilt)
    setStatusText('已发送舵机角度')
  }

  const toggleCamera = () => {
    if (!isConnected) {
      setStatusText('请先连接后再开启画面')
      return
    }
    if (!cameraOn) {
      cameraTopicRef.current = imageTopic
      subscribeCamera(imageTopic)
      setCameraOn(true)
      setStatusText('相机订阅已开启')
    } else {
      unsubscribeCamera(cameraTopicRef.current)
      setCameraOn(false)
      setImageSrc('')
      setHasRawFrame(false)
      setStatusText('相机订阅已关闭')
    }
  }

  const applyCameraTopic = () => {
    if (!isConnected) {
      setStatusText('请先连接后再订阅')
      return
    }
    if (!cameraOn) {
      setStatusText('请先开启相机画面')
      return
    }
    if (cameraTopicRef.current === imageTopic) {
      setStatusText('相机话题未变化')
      return
    }
    unsubscribeCamera(cameraTopicRef.current)
    cameraTopicRef.current = imageTopic
    subscribeCamera(imageTopic)
    setImageSrc('')
    setHasRawFrame(false)
    setStatusText('相机话题已更新')
  }

  const requestTopics = () => {
    if (!isConnected) {
      setStatusText('请先连接后再检测')
      return
    }
    const id = `topics_${Date.now()}`
    sendMessage({
      op: 'call_service',
      id,
      service: '/rosapi/topics',
      type: 'rosapi/srv/Topics',
      args: {},
    })
    setStatusText('正在获取话题列表...')
  }

  useEffect(() => {
    return () => {
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [])

  return (
    <div className="page">
      <header className="header">
        <div>
          <h1>Freenove ROS2 局域网控制台</h1>
          <p>底盘、舵机、相机统一控制</p>
        </div>
        <div className={`status ${connection}`}>{statusText}</div>
      </header>

      <section className="card connection-card">
        <div className="field">
          <label>Rosbridge IP</label>
          <input
            value={wsHost}
            onChange={(event) => setWsHost(event.target.value.trim())}
            placeholder="192.168.1.10"
          />
        </div>
        <div className="button-row">
          <button className="primary" onClick={connect}>
            连接
          </button>
          <button className="ghost" onClick={disconnect}>
            断开
          </button>
        </div>
      </section>

      <section className="card joystick-card">
        <h2>摇杆控制</h2>
        <div className="joystick-grid">
          <div className="joystick-panel">
            <div className="joystick-meta">
              <h3>底盘平移</h3>
              <span>
                前后 {linearX.toFixed(2)} / 横移 {linearY.toFixed(2)}
              </span>
            </div>
            <Joystick
              label="前后 + 横移"
              onChange={(x, y) => {
                const nextLinearX = Number((-y * MAX_LINEAR).toFixed(2))
                const nextLinearY = Number((x * MAX_LATERAL).toFixed(2))
                setLinearX(nextLinearX)
                setLinearY(nextLinearY)
                sendChassisThrottled(nextLinearX, nextLinearY, angularZ)
              }}
            />
          </div>
          <div className="joystick-panel">
            <div className="joystick-meta">
              <h3>底盘转向</h3>
              <span>角速度 {angularZ.toFixed(2)}</span>
            </div>
            <Joystick
              label="左转 / 右转"
              axis="x"
              onChange={(x) => {
                const nextAngular = Number((x * MAX_ANGULAR).toFixed(2))
                setAngularZ(nextAngular)
                sendChassisThrottled(linearX, linearY, nextAngular)
              }}
            />
          </div>
          <div className="joystick-panel">
            <div className="joystick-meta">
              <h3>舵机控制</h3>
              <span>
                左右 {servoPan}° / 上下 {servoTilt}°
              </span>
            </div>
            <Joystick
              label="左右 + 上下"
              onChange={(x, y) => {
                const nextPan = clamp(
                  Math.round(90 + x * SERVO_RANGE),
                  0,
                  180,
                )
                const nextTilt = clamp(
                  Math.round(90 - y * SERVO_RANGE),
                  0,
                  180,
                )
                setServoPan(nextPan)
                setServoTilt(nextTilt)
                sendServoThrottled(nextPan, nextTilt)
              }}
            />
          </div>
        </div>
      </section>

      <div className="grid">
        <section className="card">
          <h2>雷达探测</h2>
          <Radar range={ultrasonicRange} />
        </section>

        <section className="card">
          <h2>底盘控制</h2>
          <div className="slider-group">
            <div className="slider-row">
              <label>前后 (linear.x)</label>
              <input
                type="range"
                min="-1"
                max="1"
                step="0.05"
                value={linearX}
                onChange={(event) =>
                  setLinearX(parseFloat(event.target.value))
                }
              />
              <span>{linearX.toFixed(2)}</span>
            </div>
            <div className="slider-row">
              <label>横移 (linear.y)</label>
              <input
                type="range"
                min="-1"
                max="1"
                step="0.05"
                value={linearY}
                onChange={(event) =>
                  setLinearY(parseFloat(event.target.value))
                }
              />
              <span>{linearY.toFixed(2)}</span>
            </div>
            <div className="slider-row">
              <label>旋转 (angular.z)</label>
              <input
                type="range"
                min="-2"
                max="2"
                step="0.1"
                value={angularZ}
                onChange={(event) =>
                  setAngularZ(parseFloat(event.target.value))
                }
              />
              <span>{angularZ.toFixed(2)}</span>
            </div>
          </div>
          <div className="button-row">
            <button className="primary" onClick={sendChassis}>
              发送底盘指令
            </button>
            <button className="danger" onClick={stopChassis}>
              立即停止
            </button>
          </div>
        </section>

        <section className="card">
          <h2>舵机控制</h2>
          <div className="slider-group">
            <div className="slider-row">
              <label>左右舵机</label>
              <input
                type="range"
                min="0"
                max="180"
                step="1"
                value={servoPan}
                onChange={(event) =>
                  setServoPan(clamp(parseInt(event.target.value, 10), 0, 180))
                }
              />
              <span>{servoPan}°</span>
            </div>
            <div className="slider-row">
              <label>上下舵机</label>
              <input
                type="range"
                min="0"
                max="180"
                step="1"
                value={servoTilt}
                onChange={(event) =>
                  setServoTilt(clamp(parseInt(event.target.value, 10), 0, 180))
                }
              />
              <span>{servoTilt}°</span>
            </div>
          </div>
          <div className="button-row">
            <button className="primary" onClick={sendServo}>
              发送舵机角度
            </button>
          </div>
        </section>
      </div>

      <section className="card camera-card">
        <div className="camera-header">
          <h2>相机画面</h2>
          <button className="primary" onClick={toggleCamera}>
            {cameraOn ? '关闭画面' : '开启画面'}
          </button>
        </div>
        <div className="camera-config">
          <div className="field">
            <label>相机话题</label>
            <input
              value={imageTopic}
              onChange={(event) => setImageTopic(event.target.value)}
              placeholder="/image_raw/compressed"
            />
          </div>
          <div className="button-row">
            <button className="ghost" onClick={applyCameraTopic}>
              应用话题
            </button>
            <button className="ghost" onClick={requestTopics}>
              检测话题
            </button>
          </div>
        </div>
        <div className="camera-info">
          <div>收到帧数：{cameraStats.receiveCount}</div>
          <div>
            最近帧：{cameraStats.lastAt ? new Date(cameraStats.lastAt).toLocaleTimeString() : '无'}
          </div>
          <div>话题：{cameraStats.lastTopic || '无'}</div>
          <div>类型：{cameraStats.lastTopic ? cameraTypes[cameraStats.lastTopic] || '未知' : '无'}</div>
          <div>
            尺寸：{cameraStats.width && cameraStats.height ? `${cameraStats.width}×${cameraStats.height}` : '无'}
          </div>
          <div>编码：{cameraStats.encoding || cameraStats.format || '无'}</div>
          <div>数据大小：{cameraStats.dataSize ?? '无'}</div>
          <div>最新消息：{wsStats.lastOp || '无'}</div>
          <div>
            消息话题：{wsStats.lastTopic || '无'}
          </div>
          <div>
            最近消息：{wsStats.lastAt ? new Date(wsStats.lastAt).toLocaleTimeString() : '无'}
          </div>
          <div>消息大小：{wsStats.rawSize ?? '无'}</div>
          <div>消息总数：{wsStats.totalMessages}</div>
          <div>发送消息：{wsSendPreview || '无'}</div>
          <div>
            发现话题：
            {cameraTopics.length
              ? cameraTopics
                  .map((topic) => `${topic}${cameraTypes[topic] ? `(${cameraTypes[topic]})` : ''}`)
                  .join(', ')
              : '未获取'}
          </div>
          <div>原始消息：{wsPayloadPreview || '无'}</div>
        </div>
        <div className="camera-preview">
          {imageSrc ? (
            <img src={imageSrc} alt="camera" style={{ display: 'block' }} />
          ) : (
            <canvas
              ref={canvasRef}
              style={{ display: hasRawFrame ? 'block' : 'none' }}
            />
          )}
          {!imageSrc && !hasRawFrame ? (
            <div className="camera-placeholder">暂无画面</div>
          ) : null}
        </div>
      </section>
    </div>
  )
}

export default ControlPage
