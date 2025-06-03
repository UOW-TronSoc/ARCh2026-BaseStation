import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import MainNavbar from 'components/MainNavbar/MainNavbar';
import Dashboard from 'pages/Dashboard/Dashboard';
import ScriptManager from 'pages/ScriptManager/ScriptManager';
import ArmControl from 'pages/ArmControl/ArmControl';
import CameraFeed from 'pages/CameraFeed/CameraFeed';
import LogViewer from 'pages/LogViewer/LogViewer';

import './styles/variables.css';
import './App.css';



export default function App() {
  return (
    <Router>
      <MainNavbar />
      <Routes>
        <Route path="/"          element={<Dashboard />} />
        <Route path="/dashboard" element={<Dashboard />} />
        <Route path="/arm-control" element={<ArmControl />} />
        <Route path="/script-manager" element={<ScriptManager />} />
        <Route path="/cameras" element={<CameraFeed />} />
        <Route path="/logs" element={<LogViewer />} />
        <Route path="*"          element={<div className="text-center mt-4">404 Not Found</div>} />
      </Routes>
    </Router>
  );
}
