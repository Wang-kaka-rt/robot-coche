import { useCallback, useEffect, useRef, useState } from 'react'
import * as ROSLIB from 'roslib'
import './ControlPage.css' // Reuse control page styles for now

function ChatPage() {
  const [wsHost, setWsHost] = useState(localStorage.getItem('robot_ip') || '')
  const [llmHost, setLlmHost] = useState(localStorage.getItem('llm_host') || 'http://localhost:1234/v1/chat/completions')
  const [apiKey, setApiKey] = useState(localStorage.getItem('llm_api_key') || 'lm-studio')
  const [connection, setConnection] = useState('disconnected')
  const [statusText, setStatusText] = useState('未连接')
  
  // Camera State
  const [imageSrc, setImageSrc] = useState('')
  
  // Chat State
  const [chatMessages, setChatMessages] = useState([])
  const [chatInput, setChatInput] = useState('')
  const [isSending, setIsSending] = useState(false)

  const rosRef = useRef(null)
  const cameraListenerRef = useRef(null)
  const cmdVelRef = useRef(null)

  // Load IP from local storage on mount
  useEffect(() => {
    const savedIp = localStorage.getItem('robot_ip')
    if (savedIp) {
        setWsHost(savedIp)
    }
  }, [])

  const subscribeCamera = useCallback((topic) => {
    if (!rosRef.current) return

    if (cameraListenerRef.current) {
      cameraListenerRef.current.unsubscribe()
    }

    const listener = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: 'sensor_msgs/CompressedImage'
    })

    listener.subscribe((message) => {
      if (message.format && message.data) {
        setImageSrc('data:image/jpeg;base64,' + message.data)
      }
    })

    cameraListenerRef.current = listener
  }, [])

  const connect = useCallback(() => {
    if (!wsHost) {
      setStatusText('请输入机器人IP地址')
      return
    }
    
    // Save IP
    localStorage.setItem('robot_ip', wsHost)

    if (rosRef.current) {
      rosRef.current.close()
    }
    setConnection('connecting')
    setStatusText('连接中...')
    
    let url = wsHost
    if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
        url = `ws://${url}`
    }
    if (!url.split(':')[2]) {
        url = `${url}:9090`
    }

    const ros = new ROSLIB.Ros({ url: url })

    ros.on('connection', () => {
      setConnection('connected')
      setStatusText('已连接')
      // Auto start camera on connect
      subscribeCamera('/image_raw/compressed')
      
      // Init cmd_vel publisher
      cmdVelRef.current = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
      })
    })

    ros.on('error', () => {
      setConnection('error')
      setStatusText('连接错误')
    })

    ros.on('close', () => {
      setConnection('disconnected')
      setStatusText('已断开')
    })

    rosRef.current = ros
  }, [subscribeCamera, wsHost])

  useEffect(() => {
    if (wsHost && connection === 'disconnected') {
      connect()
    }
  }, [wsHost, connection, connect])

  useEffect(() => {
    return () => {
      if (rosRef.current) rosRef.current.close()
    }
  }, [])

  const executeCommand = (action) => {
    if (!cmdVelRef.current) return

    let linearX = 0.0
    let angularZ = 0.0

    if (action.includes('forward') || action.includes('front')) {
        linearX = 0.3
    } else if (action.includes('back')) {
        linearX = -0.3
    } else if (action.includes('left')) {
        angularZ = 0.5
    } else if (action.includes('right')) {
        angularZ = -0.5
    }

    const twist = {
        linear: { x: linearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularZ }
    }
    cmdVelRef.current.publish(twist)

    // Stop after 1s
    if (linearX !== 0 || angularZ !== 0) {
        setTimeout(() => {
            cmdVelRef.current.publish({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            })
        }, 1000)
    }
  }

  const handleSendMessage = async () => {
    if (!chatInput.trim() || isSending) return

    const userMsg = { role: 'user', content: chatInput }
    setChatMessages(prev => [...prev, userMsg])
    setChatInput('')
    setIsSending(true)

    try {
      // 1. Prepare messages for LM Studio
      const messages = [
        {
            role: "system",
            content: "You are a robot assistant. Answer concisely. If user asks to move, output JSON: {\"action\": \"move_forward\"/\"move_backward\"/\"turn_left\"/\"turn_right\"}. Otherwise just answer."
        },
        {
            role: "user",
            content: [
                { type: "text", text: userMsg.content }
            ]
        }
      ]

      // 2. Attach image if available
      if (imageSrc) {
        messages[1].content.push({
            type: "image_url",
            image_url: {
                url: imageSrc
            }
        })
      }

      // 3. Send direct to LM Studio (CORS must be enabled in LM Studio)
      const headers = { 
        'Content-Type': 'application/json'
      }
      
      // Only attach Authorization if API Key is provided
      if (apiKey) {
          headers['Authorization'] = `Bearer ${apiKey}`
      }

      const response = await fetch(llmHost, {
        method: 'POST',
        headers: headers,
        body: JSON.stringify({
          model: "local-model",
          messages: messages,
          temperature: 0.7
        })
      })

      if (!response.ok) {
          const errText = await response.text()
          throw new Error(`LM Studio Error (${response.status}): ${errText}`)
      }

      const data = await response.json()
      const content = data.choices[0].message.content
      
      setChatMessages(prev => [...prev, { role: 'robot', content: content }])

      // 4. Parse commands locally
      if (content.includes('{') && content.includes('}')) {
          try {
              const jsonStr = content.substring(content.indexOf('{'), content.lastIndexOf('}') + 1)
              const cmd = JSON.parse(jsonStr)
              if (cmd.action) {
                  executeCommand(cmd.action)
              }
          } catch (e) {
              console.error("JSON parse error", e)
          }
      } else {
          // Simple keyword fallback
          const lower = content.toLowerCase()
          if (lower.includes('move forward')) executeCommand('forward')
          else if (lower.includes('move backward')) executeCommand('back')
          else if (lower.includes('turn left')) executeCommand('left')
          else if (lower.includes('turn right')) executeCommand('right')
      }

    } catch (err) {
      setChatMessages(prev => [...prev, { role: 'system', content: `Error: ${err.message}. Ensure LM Studio CORS is enabled!` }])
    } finally {
      setIsSending(false)
    }
  }

  return (
    <div className="container" style={{ maxWidth: '800px', margin: '0 auto', padding: '20px' }}>
      <header className="header" style={{ marginBottom: '20px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <h1>AI 视觉对话</h1>
        <div className="connection-status">
            <span className={`status-indicator status-${connection}`} />
            <span style={{ marginRight: '10px' }}>{statusText}</span>
            <input 
                value={wsHost} 
                onChange={e => {
                    setWsHost(e.target.value)
                    localStorage.setItem('robot_ip', e.target.value)
                }} 
                placeholder="Robot IP"
                title="Robot WebSocket IP"
                style={{ width: '120px', padding: '5px', borderRadius: '4px', border: '1px solid #444', background: '#222', color: 'white', marginRight: '5px' }}
            />
            <input 
                value={llmHost} 
                onChange={e => {
                    setLlmHost(e.target.value)
                    localStorage.setItem('llm_host', e.target.value)
                }} 
                placeholder="LLM URL"
                title="LM Studio API URL"
                style={{ width: '200px', padding: '5px', borderRadius: '4px', border: '1px solid #444', background: '#222', color: 'white', marginRight: '5px' }}
            />
            <input 
                value={apiKey} 
                onChange={e => {
                    setApiKey(e.target.value)
                    localStorage.setItem('llm_api_key', e.target.value)
                }} 
                placeholder="API Key"
                title="LM Studio API Key (Leave empty if disabled)"
                type="password"
                style={{ width: '80px', padding: '5px', borderRadius: '4px', border: '1px solid #444', background: '#222', color: 'white' }}
            />
            <button onClick={connect} style={{ marginLeft: '10px', padding: '5px 15px' }}>连接</button>
        </div>
      </header>

      <div className="main-content" style={{ display: 'flex', flexDirection: 'column', gap: '20px' }}>
        <section className="card camera-card" style={{ padding: '0', overflow: 'hidden' }}>
            <div className="camera-preview" style={{ background: '#000', minHeight: '300px', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                {imageSrc ? (
                    <img src={imageSrc} alt="camera" style={{ width: '100%', display: 'block' }} />
                ) : (
                    <div style={{ color: '#666' }}>等待相机画面...</div>
                )}
            </div>
        </section>

        <section className="card chat-card" style={{ display: 'flex', flexDirection: 'column', height: '400px' }}>
            <div className="chat-history" style={{ 
                flex: 1,
                overflowY: 'auto', 
                background: '#1a1a1a', 
                padding: '20px',
                borderRadius: '8px',
                display: 'flex',
                flexDirection: 'column',
                gap: '15px'
            }}>
                {chatMessages.length === 0 && (
                    <div style={{ textAlign: 'center', color: '#666', marginTop: '50px' }}>
                        👋 你好！我是你的机器人助手。<br/>我可以看见你看到的世界，快问我问题吧！
                    </div>
                )}
                {chatMessages.map((msg, idx) => (
                    <div key={idx} style={{ 
                        alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
                        maxWidth: '80%',
                        display: 'flex',
                        flexDirection: 'column',
                        alignItems: msg.role === 'user' ? 'flex-end' : 'flex-start'
                    }}>
                        <div style={{ fontSize: '12px', color: '#888', marginBottom: '4px' }}>
                            {msg.role === 'user' ? 'Me' : 'Robot'}
                        </div>
                        <div style={{ 
                            background: msg.role === 'user' ? '#007acc' : '#333',
                            color: 'white',
                            padding: '10px 15px',
                            borderRadius: '12px',
                            borderTopRightRadius: msg.role === 'user' ? '2px' : '12px',
                            borderTopLeftRadius: msg.role === 'robot' ? '2px' : '12px',
                            lineHeight: '1.5'
                        }}>
                            {msg.content}
                        </div>
                    </div>
                ))}
            </div>
            
            <div className="chat-input-area" style={{ marginTop: '15px', display: 'flex', gap: '10px' }}>
                <input 
                    value={chatInput}
                    onChange={e => setChatInput(e.target.value)}
                    onKeyPress={e => e.key === 'Enter' && handleSendMessage()}
                    placeholder="输入问题..."
                    disabled={isSending}
                    style={{ 
                        flex: 1, 
                        padding: '12px', 
                        borderRadius: '8px', 
                        border: '1px solid #444', 
                        background: '#222', 
                        color: 'white',
                        fontSize: '16px'
                    }}
                />
                <button 
                    className="primary" 
                    onClick={handleSendMessage}
                    disabled={isSending}
                    style={{ padding: '0 25px', fontSize: '16px', fontWeight: 'bold' }}
                >
                    {isSending ? '...' : '发送'}
                </button>
            </div>
        </section>
      </div>
    </div>
  )
}

export default ChatPage
