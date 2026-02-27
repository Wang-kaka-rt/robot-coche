import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  define: {
    // 兼容 roslib 等依赖 global 变量的库
    global: 'window',
  },
})
