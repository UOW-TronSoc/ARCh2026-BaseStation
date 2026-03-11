import { BrowserRouter as Router, Routes, Route, useLocation } from 'react-router-dom';
import { AuthProvider } from 'context/AuthContext';
import { BatteryProvider } from 'context/BatteryContext';
import ProtectedRoute from 'components/ProtectedRoute/ProtectedRoute';
import MainNavbar from 'components/MainNavbar/MainNavbar';
import PinPage from 'pages/PinPage/PinPage';
import Dashboard from 'pages/Dashboard/Dashboard';
import ScriptManager from 'pages/ScriptManager/ScriptManager';
import ArmControl from 'pages/ArmControl/ArmControl';
import CameraFeed from 'pages/CameraFeed/CameraFeed';
import LogViewer from 'pages/LogViewer/LogViewer';
import Checklist from 'pages/Checklist/Checklist';
import AutoMap from 'pages/AutoMap/AutoMap';
import Science from 'pages/Science/Science';

import './styles/variables.css';
import './App.css';

function AppContent() {
  const location = useLocation();
  const isPinPage = location.pathname === '/pin';

  return (
    <div className={`appLayout ${isPinPage ? 'appLayout--pin' : ''}`}>
      {!isPinPage && <MainNavbar />}
      <main className="appMain">
        <div className="appMain-inner">
          <Routes>
        <Route path="/pin" element={<PinPage />} />
        <Route path="/" element={<ProtectedRoute><Dashboard /></ProtectedRoute>} />
        <Route path="/dashboard" element={<ProtectedRoute><Dashboard /></ProtectedRoute>} />
        <Route path="/arm-control" element={<ProtectedRoute><ArmControl /></ProtectedRoute>} />
        <Route path="/script-manager" element={<ProtectedRoute><ScriptManager /></ProtectedRoute>} />
        <Route path="/cameras" element={<ProtectedRoute><CameraFeed /></ProtectedRoute>} />
        <Route path="/checklist" element={<ProtectedRoute><Checklist /></ProtectedRoute>} />
        <Route path="/automap" element={<ProtectedRoute><AutoMap /></ProtectedRoute>} />
        <Route path="/science" element={<ProtectedRoute><Science /></ProtectedRoute>} />
        <Route path="/logs" element={<ProtectedRoute><LogViewer /></ProtectedRoute>} />
        <Route path="*" element={<ProtectedRoute><div className="text-center mt-4">404 Not Found</div></ProtectedRoute>} />
          </Routes>
        </div>
      </main>
    </div>
  );
}

export default function App() {
  return (
    <AuthProvider>
      <BatteryProvider>
        <Router>
          <AppContent />
        </Router>
      </BatteryProvider>
    </AuthProvider>
  );
}
