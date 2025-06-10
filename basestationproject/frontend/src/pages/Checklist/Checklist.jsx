import React, { useState, useEffect } from "react";
import axios from "axios";
import "./Checklist.css";

const ChecklistPage = () => {
  document.title = "Checklist"
  const [groups, setGroups] = useState([]);
  const [selectedGroupId, setSelectedGroupId] = useState(null);
  const [checklist, setChecklist] = useState([]);
  const [newTaskText, setNewTaskText] = useState("");
  const [showFields, setShowFields] = useState({});

  const API_BASE = "http://127.0.0.1:8000/api";

  const topLevelTasks = checklist.filter((task) => task.parent === null);
  const getSubtasks = (parentId) =>
    checklist.filter((task) => task.parent === parentId);

  useEffect(() => {
    axios.get(`${API_BASE}/checklist/groups/`).then((res) => {
      setGroups(res.data);
      if (res.data.length > 0) {
        setSelectedGroupId(res.data[0].id);
        setChecklist(res.data[0].tasks);
      }
    });
  }, []);

  useEffect(() => {
    if (selectedGroupId) {
      axios
        .get(`${API_BASE}/checklist/groups/${selectedGroupId}/`)
        .then((res) => {
          setChecklist(res.data.tasks);
        });
    }
  }, [selectedGroupId]);

  const toggleCompletion = (task) => {
    const updated = { ...task, completed: !task.completed };
    axios.put(`${API_BASE}/checklist/tasks/${task.id}/`, updated).then(() => {
      setChecklist((prev) =>
        prev.map((t) =>
          t.id === task.id ? { ...t, completed: !t.completed } : t
        )
      );
    });
  };

  const updateValue = (task, value) => {
    axios
      .put(`${API_BASE}/checklist/tasks/${task.id}/`, { ...task, value })
      .then(() => {
        setChecklist((prev) =>
          prev.map((t) => (t.id === task.id ? { ...t, value } : t))
        );
      });
  };

  const addTask = () => {
    if (!newTaskText.trim()) return;
    axios
      .post(`${API_BASE}/checklist/tasks/`, {
        group: selectedGroupId,
        text: newTaskText,
        completed: false,
        value: "",
        parent: null,
      })
      .then((res) => {
        setChecklist([...checklist, res.data]);
        setNewTaskText("");
      });
  };

  const addSubtask = (parentId) => {
    const text = prompt("Enter subtask text");
    if (!text) return;
    axios
      .post(`${API_BASE}/checklist/tasks/`, {
        group: selectedGroupId,
        text,
        completed: false,
        value: "",
        parent: parentId,
      })
      .then((res) => {
        setChecklist([...checklist, res.data]);
      });
  };

  const deleteTask = (taskId) => {
    axios.delete(`${API_BASE}/checklist/tasks/${taskId}/`).then(() => {
      setChecklist(checklist.filter((task) => task.id !== taskId));
    });
  };

  const toggleField = (id) => {
    setShowFields((prev) => ({ ...prev, [id]: !prev[id] }));
  };

  const renderTask = (task, depth = 0) => (
    <li
      key={task.id}
      className={`list-group-item ${
        task.completed ? "list-group-item-success" : ""
      }`}
      style={{
        marginLeft: depth * 20,
        marginBottom: "1rem",
        borderRadius: "8px",
        padding: "1rem",
        backgroundColor: "var(--card-alt-bg)",
        color: "var(--card-alt-fg)",
      }}
    >
      <div className="d-flex justify-content-between align-items-start">
        <span
          onClick={() => toggleCompletion(task)}
          style={{
            cursor: "pointer",
            fontWeight: "500",
            fontSize: "1rem",
            textDecoration: task.completed ? "line-through" : "none",
            color: task.completed ? "#ff4d4f" : "inherit", // Danger red
          }}
        >
          {task.text}
        </span>
      </div>

      {showFields[task.id] && (
        <input
          className="form-control mt-3"
          value={task.value || ""}
          onChange={(e) => updateValue(task, e.target.value)}
          placeholder="Enter a note or value..."
          style={{
            backgroundColor: "#2b2b2b",
            border: "1px solid #444",
            color: "#fff",
          }}
        />
      )}

      <div className="d-flex flex-wrap mt-3 gap-2">
        <button
          className="btn btn-sm btn-outline-secondary"
          onClick={() => toggleField(task.id)}
        >
          {showFields[task.id] ? "Hide Field" : "Add Field"}
        </button>

        <button
          className="btn btn-sm btn-outline-light"
          onClick={() => addSubtask(task.id)}
        >
          + Add Subtask
        </button>

        <button
          className="btn btn-sm btn-danger"
          onClick={() => {
            const confirmDelete = window.confirm(
              "Are you sure you want to delete this task?"
            );
            if (confirmDelete) deleteTask(task.id);
          }}
        >
          ✕ Delete
        </button>
      </div>

      <ul className="list-group mt-3">
        {getSubtasks(task.id).map((subtask) => renderTask(subtask, depth + 1))}
      </ul>
    </li>
  );

  return (
    <div className="container checklist-container mt-4">
      <h2>Day Checklist</h2>

      <select
        className="form-select mb-3"
        value={selectedGroupId || ""}
        onChange={(e) => setSelectedGroupId(parseInt(e.target.value))}
      >
        {groups.map((group) => (
          <option key={group.id} value={group.id}>
            {group.name}
          </option>
        ))}
      </select>

      <div className="mb-3 d-flex">
        <input
          className="form-control me-2"
          placeholder="Add a new task..."
          value={newTaskText}
          onChange={(e) => setNewTaskText(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") {
              e.preventDefault();
              addTask();
            }
          }}
        />
        <button className="btn btn-success" onClick={addTask}>
          Add
        </button>
      </div>

      <ul className="list-group">
        {topLevelTasks.map((task) => renderTask(task))}
      </ul>
    </div>
  );
};

export default ChecklistPage;
