import { BrowserRouter, Routes, Route, Link } from 'react-router-dom'
import ControlPage from './page/ControlPage.jsx'
import ChatPage from './page/ChatPage.jsx'
import './App.css'

function App() {
  return (
    <BrowserRouter>
      <div className="app-nav" style={{ 
          position: 'fixed', 
          top: 0, 
          left: 0, 
          right: 0, 
          background: '#1a1a1a', 
          padding: '10px 20px', 
          zIndex: 1000,
          borderBottom: '1px solid #333',
          display: 'flex',
          justifyContent: 'center',
          gap: '20px'
      }}>
        <Link to="/" style={{ color: 'white', textDecoration: 'none', fontWeight: 'bold' }}>遥控模式</Link>
        <Link to="/chat" style={{ color: 'white', textDecoration: 'none', fontWeight: 'bold' }}>AI 对话</Link>
      </div>
      
      <div style={{ paddingTop: '60px' }}>
        <Routes>
          <Route path="/" element={<ControlPage />} />
          <Route path="/chat" element={<ChatPage />} />
        </Routes>
      </div>
    </BrowserRouter>
  )
}

export default App
