const TASK_CHECKLISTS = {
  "Post-Landing Task": {
    sections: {
      Setup: [
        "Briefing attendance confirmed",
        "E-Stop and signal lights tested",
        "Cameras initialized and diagnostics run",
      ],
      Execution: [
        "Descend down ramp",
        "Circumnavigate lander",
        "Visual inspection for damage",
        "RFID site navigation",
        "Perform maintenance tasks",
        "Connect hose to lander and plant",
      ],
      Presentation: [
        "Prepare 5–10 slides summary",
        "Report findings to judges",
      ],
    },
  },
  "Space Resources Task": {
    sections: {
      Setup: [
        "Briefing complete",
        "Science sensors calibrated",
        "Processing unit installed",
      ],
      Execution: [
        "Navigate to 2 ice and 2 ilmenite sites",
        "Take measurements",
        "Excavate & process sample",
        "Submit container to judges",
      ],
      Presentation: [
        "Quantify water & ilmenite wt%",
        "Explain prospecting tools & methods",
      ],
    },
  },
  // Add others: Excavation & Construction, Mapping, etc.
};

export default TASK_CHECKLISTS;
