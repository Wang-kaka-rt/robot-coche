import { useEffect, useRef, useState } from 'react'
import * as ROSLIB from 'roslib'
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
  // 默认为 /image_raw/compressed，与后端 camera_node.py 保持一致
  const [imageTopic, setImageTopic] = useState('/image_raw/compressed')
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
  const [ultrasonicRange, setUltrasonicRange] = useState(null)
  const [infraredValue, setInfraredValue] = useState(0)
  
  const rosRef = useRef(null)
  const cmdVelRef = useRef(null)
  const servoPanRef = useRef(null)
  const servoTiltRef = useRef(null)
  const cameraListenerRef = useRef(null)
  
  const canvasRef = useRef(null)
  const chassisSendAtRef = useRef(0)
  const servoSendAtRef = useRef(0)
  const cameraTopicRef = useRef('/image_raw')

  const isConnected = connection === 'connected'

  const socketReady = () =>
    rosRef.current && rosRef.current.isConnected

  const publishChassis = (x, y, z) => {
    if (!cmdVelRef.current) return
    const twist = buildTwist(x, y, z)
    cmdVelRef.current.publish(twist)
  }

  const publishServo = (pan, tilt) => {
    if (servoPanRef.current) {
        servoPanRef.current.publish({ data: pan })
    }
    if (servoTiltRef.current) {
        servoTiltRef.current.publish({ data: tilt })
    }
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

  const resolveImageSrc = (msg) => {
    // ROSLIB already handles decoding if we use it correctly, but for CompressedImage
    // msg.data is usually a Base64 string in JSON mode.
    if (!msg || !msg.data) {
      return ''
    }
    let format = String(msg.format || 'jpeg').toLowerCase()
    if (format.includes(';')) {
        format = format.split(';')[0].trim()
    }
    
    let mime = 'image/jpeg'
    if (format.includes('png')) {
      mime = 'image/png'
    } else if (format.includes('gif')) {
        mime = 'image/gif'
    } else if (format.includes('bmp')) {
        mime = 'image/bmp'
    } else if (format.includes('webp')) {
        mime = 'image/webp'
    } else if (format.includes('svg')) {
        mime = 'image/svg+xml'
    }
    
    // msg.data is Base64 string from roslib (if using standard rosbridge JSON)
    return `data:${mime};base64,${msg.data}`
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
    
    // msg.data in roslib for 'uint8[]' is Base64 string
    const binary = atob(msg.data)
    const len = binary.length
    const data = new Uint8Array(len)
    for (let i = 0; i < len; i++) {
        data[i] = binary.charCodeAt(i)
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
    const dataSize = typeof msg.data === 'string' ? msg.data.length : 0
    
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
    
    // CompressedImage usually has 'format'
    if (msg.format || (!msg.height && !msg.width)) {
      const src = resolveImageSrc(msg)
      if (src) {
        setImageSrc(src)
        setHasRawFrame(false)
        return true
      }
    } 
    // Raw Image has width/height/encoding
    else if (msg.height && msg.width && msg.encoding) {
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

  const subscribeCamera = (topic) => {
    if (!rosRef.current) return

    // Unsubscribe previous
    if (cameraListenerRef.current) {
        cameraListenerRef.current.unsubscribe()
        cameraListenerRef.current = null
    }

    const normalized = normalizeTopic(topic)
    const topicName = withLeadingSlash(normalized)
    
    // Auto-detect type based on name (simple heuristic)
    // Ideally we should use rosapi to check type, but user input is faster
    let messageType = 'sensor_msgs/Image'
    if (topicName.includes('compressed')) {
        messageType = 'sensor_msgs/CompressedImage'
    }

    const listener = new ROSLIB.Topic({
        ros: rosRef.current,
        name: topicName,
        messageType: messageType
    })

    listener.subscribe((message) => {
        renderCameraMessage(message, topicName)
        setWsStats(prev => ({
            ...prev,
            lastTopic: topicName,
            lastAt: Date.now(),
            totalMessages: prev.totalMessages + 1
        }))
    })

    cameraListenerRef.current = listener
    cameraTopicRef.current = topicName
  }

  const unsubscribeCamera = () => {
    if (cameraListenerRef.current) {
        cameraListenerRef.current.unsubscribe()
        cameraListenerRef.current = null
    }
  }

  const connect = () => {
    if (!wsHost) {
      setStatusText('请输入机器人IP地址')
      return
    }

    if (rosRef.current) {
      rosRef.current.close()
    }
    setConnection('connecting')
    setStatusText('连接中...')
    
    // Handle user input that might already include protocol or port
    let url = wsHost
    if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
        url = `ws://${url}`
    }
    if (!url.split(':')[2]) {
        url = `${url}:9090`
    }

    const ros = new ROSLIB.Ros({
        url: url
    })

    ros.on('connection', () => {
      setConnection('connected')
      setStatusText('已连接')
      
      // Setup publishers
      cmdVelRef.current = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
      })

      servoPanRef.current = new ROSLIB.Topic({
        ros: ros,
        name: '/servo_angle',
        messageType: 'std_msgs/Int32'
      })

      servoTiltRef.current = new ROSLIB.Topic({
        ros: ros,
        name: '/servo_angle_tilt',
        messageType: 'std_msgs/Int32'
      })

      // Subscribe to sensors
      const ultrasonicListener = new ROSLIB.Topic({
        ros: ros,
        name: '/ultrasonic/range',
        messageType: 'sensor_msgs/Range'
      })
      ultrasonicListener.subscribe((msg) => {
          setUltrasonicRange(msg.range)
      })

      const lineListener = new ROSLIB.Topic({
        ros: ros,
        name: '/line_sensor',
        messageType: 'std_msgs/UInt8'
      })
      lineListener.subscribe((msg) => {
          setInfraredValue(msg.data)
      })

      if (cameraOn) {
        subscribeCamera(imageTopic)
      }
    })

    ros.on('error', (error) => {
      setConnection('disconnected')
      setStatusText('连接失败')
      console.error('ROS Error:', error)
    })

    ros.on('close', () => {
      setConnection('disconnected')
      setStatusText('已断开')
      setImageSrc('')
      setHasRawFrame(false)
      setCameraTopics([])
      setCameraTypes({})
    })

    rosRef.current = ros
  }

  const disconnect = () => {
    if (rosRef.current) {
      rosRef.current.close()
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
      subscribeCamera(imageTopic)
      setCameraOn(true)
      setStatusText('相机订阅已开启')
    } else {
      unsubscribeCamera()
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
    subscribeCamera(imageTopic)
    setImageSrc('')
    setHasRawFrame(false)
    setStatusText('相机话题已更新/重置')
  }

  const requestTopics = () => {
    if (!isConnected) {
      setStatusText('请先连接后再检测')
      return
    }
    
    const topicsClient = new ROSLIB.Service({
        ros: rosRef.current,
        name: '/rosapi/topics',
        serviceType: 'rosapi/srv/Topics'
    })

    setStatusText('正在获取话题列表...')
    topicsClient.callService({}, (result) => {
        const topics = result.topics || []
        const types = result.types || []
        const mapping = {}
        topics.forEach((topic, index) => {
            mapping[topic] = types[index] || ''
        })
        setCameraTopics(topics)
        setCameraTypes(mapping)
        setStatusText('话题列表已更新')
    }, (error) => {
        setStatusText('获取话题失败')
        console.error(error)
    })
  }

  useEffect(() => {
    return () => {
      if (rosRef.current) {
        rosRef.current.close()
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
          <h2>环境感知</h2>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '20px', alignItems: 'center' }}>
            <Radar range={ultrasonicRange} />
            <InfraredSensor value={infraredValue} />
          </div>
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
          <div>发送消息：(roslib 自动处理)</div>
          <div>
            发现话题：
            {cameraTopics.length
              ? cameraTopics
                  .map((topic) => `${topic}${cameraTypes[topic] ? `(${cameraTypes[topic]})` : ''}`)
                  .join(', ')
              : '未获取'}
          </div>
          <div>(原始消息由 roslib 处理)</div>
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
