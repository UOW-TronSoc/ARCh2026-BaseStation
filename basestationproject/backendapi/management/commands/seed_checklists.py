"""
Management command to seed the ARCh 2026 competition checklists into the database.
Source: Competition Task Checklists.pdf

Usage:
    python manage.py seed_checklists            # insert (skips groups that already exist)
    python manage.py seed_checklists --reset    # delete all groups first, then re-insert
"""

from django.core.management.base import BaseCommand
from backendapi.models import ChecklistGroup, ChecklistTask


CHECKLISTS = [
    {
        "name": "Day 1 | Post Landing Task",
        "sections": [
            {
                "heading": "Pre-Task Setup",
                "tasks": [
                    "T-30 to T-20 mins: Task brief and weigh in",
                    "T-15 mins: Base Station setup, Antenna setup, Rover on side of field",
                    "T-5 mins: Comm setup, Rover on field",
                    "Attach connectors to lander and hoses",
                ],
            },
            {
                "heading": "Activity 1: Systems Check — 20 pts",
                "tasks": [
                    "Descent down ramp (all 4 wheels on sand) — 5 pts",
                    "Circumnavigate lander — 10 pts",
                    "Notify judges of any damage to lander — 5 pts",
                ],
            },
            {
                "heading": "Activity 2: Site Evaluation — 30 pts",
                "tasks": [
                    "Navigate to and relay status readout of supply cache 1 — 10 pts",
                    "Navigate to and relay status readout of supply cache 2 — 10 pts",
                    "Navigate to and relay status readout of supply cache 3 — 10 pts",
                ],
            },
            {
                "heading": "Activity 3: Processing Plant Maintenance — 30 pts",
                "tasks": [
                    "Maintenance job 1 (provided by judges) — 5 pts",
                    "Maintenance job 2 (provided by judges) — 5 pts",
                    "Maintenance job 3 (provided by judges) — 5 pts",
                    "Maintenance job 4 (provided by judges) — 5 pts",
                    "Maintenance job 5 (provided by judges) — 5 pts",
                    "Maintenance job 6 (provided by judges) — 5 pts",
                ],
            },
            {
                "heading": "Activity 4: Propellant Hose Connection — 10 pts",
                "tasks": [
                    "Connect hose to lander — 5 pts contact + 5 pts full connection",
                ],
            },
            {
                "heading": "Activity 5: Modular Propellant Hose Connection — 10 pts",
                "tasks": [
                    "Connect hose to processing plant — 5 pts contact + 5 pts full connection",
                ],
            },
            {
                "heading": "Post-Task",
                "tasks": [
                    "Rover carried off field",
                    "Sweep tracks and reset equipment",
                    "Remove connectors from lander and hoses",
                    "Clear Base Station",
                ],
            },
        ],
    },
    {
        "name": "Day 2 | Space Resources Task",
        "sections": [
            {
                "heading": "Pre-Task Setup",
                "tasks": [
                    "T-30 to T-20 mins: Task brief and weigh in",
                    "T-15 mins: Base Station setup, Antenna setup, Rover on side of field",
                    "T-5 mins: Comm setup, Rover on field",
                    "Weigh empty water extractor (baseline for after-task comparison)",
                ],
            },
            {
                "heading": "Activity 1: Prospecting — 8 pts",
                "tasks": [
                    "Image ilmenite site 1 and relay to judges — 2 pts",
                    "Image ilmenite site 2 and relay to judges — 2 pts",
                    "Image ice site 1 and relay to judges — 2 pts",
                    "Image ice site 2 and relay to judges — 2 pts",
                    "Quantify ice and ilmenite content for presentation — up to 20 pts",
                ],
            },
            {
                "heading": "Activity 2: Excavation & Processing — 50 pts",
                "tasks": [
                    "Collect at least one drop of water — 30 pts",
                    "Collect bulk water (ratio vs other teams) — up to 20 pts",
                    "NOTE: Manual loading available with 60% point penalty",
                ],
            },
            {
                "heading": "Activity 3: Space Resources Presentation — 42 pts (10 mins, 5–10 slides)",
                "tasks": [
                    "Explain prospecting tools used and data validity — up to 5 pts",
                    "High-quality photo: ice site 1 (in focus, individual grains, scale) — 3 pts",
                    "High-quality photo: ice site 2 (in focus, individual grains, scale) — 3 pts",
                    "High-quality photo: ilmenite site 1 (in focus, individual grains, scale) — 3 pts",
                    "High-quality photo: ilmenite site 2 (in focus, individual grains, scale) — 3 pts",
                    "Water wt% measurement: ice site 1 — up to 5 pts",
                    "Water wt% measurement: ice site 2 — up to 5 pts",
                    "Ilmenite wt% measurement: ilmenite site 1 — up to 5 pts",
                    "Ilmenite wt% measurement: ilmenite site 2 — up to 5 pts",
                    "Explain excavation and processing methods and justification — up to 5 pts",
                ],
            },
        ],
    },
    {
        "name": "Day 3 | Mapping & Autonomous Task",
        "sections": [
            {
                "heading": "Pre-Task Setup",
                "tasks": [
                    "T-30 to T-20 mins: Task brief and weigh in",
                    "T-15 mins: Base Station setup, Antenna setup, Rover on side of field",
                    "T-5 mins: Comm setup, Rover on field",
                ],
            },
            {
                "heading": "Activity 1: Leave Start Area — 5 pts",
                "tasks": [
                    "All 4 wheels must leave start square — 5 pts",
                ],
            },
            {
                "heading": "Activity 2: Autonomous Landmark Navigation — 30 pts (MUST BE AUTONOMOUS)",
                "tasks": [
                    "Traverse to, image, and relay placard 1 to base station judge — 6 pts",
                    "Traverse to, image, and relay placard 2 to base station judge — 6 pts",
                    "Traverse to, image, and relay placard 3 to base station judge — 6 pts",
                    "Traverse to, image, and relay placard 4 to base station judge — 6 pts",
                    "Traverse to, image, and relay placard 5 to base station judge — 6 pts",
                ],
            },
            {
                "heading": "Activity 3: Exploratory Mapping — 40 pts",
                "tasks": [
                    "Leave the provided map area",
                    "Report block position 1 (auto+300mm: 10pts | auto+600mm: 5pts | manual+300mm: 5pts | manual+600mm: 2pts)",
                    "Report block position 2 (auto+300mm: 10pts | auto+600mm: 5pts | manual+300mm: 5pts | manual+600mm: 2pts)",
                    "Report block position 3 (auto+300mm: 10pts | auto+600mm: 5pts | manual+300mm: 5pts | manual+600mm: 2pts)",
                    "Report block position 4 (auto+300mm: 10pts | auto+600mm: 5pts | manual+300mm: 5pts | manual+600mm: 2pts)",
                ],
            },
            {
                "heading": "Activity 4: Autonomous & Mapping Presentation — 25 pts (10 mins, 5–10 slides)",
                "tasks": [
                    "Design and justification of autonomous landmark navigation system, advantages and limitations — up to 5 pts",
                    "Design and justification of navigation component for exploratory mapping, incl. autonomous vs non-autonomous rationale — up to 5 pts",
                    "Map visualisation design and format justification — up to 5 pts",
                    "Map quality: coverage, completeness, resolution, accuracy — up to 10 pts",
                ],
            },
        ],
    },
    {
        "name": "Day 4 | Excavation & Construction Task",
        "sections": [
            {
                "heading": "Pre-Task Setup",
                "tasks": [
                    "T-30 to T-20 mins: Task brief and weigh in",
                    "T-15 mins: Base Station setup, Antenna setup, Rover on side of field",
                    "T-5 mins: Comm setup, Rover on field",
                    "Place pavers on field",
                ],
            },
            {
                "heading": "Activity 1: Ramp Descent — 5 pts",
                "tasks": [
                    "Descent down ramp (all 4 wheels on sand) — 5 pts",
                ],
            },
            {
                "heading": "Activity 2: Rock Clearing — 30 pts",
                "tasks": [
                    "Move small rock 1 (<1 kg) into collection area — 2 pts move + 2 pts in collection",
                    "Move small rock 2 (<1 kg) into collection area — 2 pts move + 2 pts in collection",
                    "Move medium rock 1 (<3 kg) into collection area — 2 pts move + 2 pts in collection",
                    "Move medium rock 2 (<3 kg) into collection area — 2 pts move + 2 pts in collection",
                    "Move large rock (<5 kg) into collection area — 3 pts move + 4 pts in collection",
                    "Move huge rock (<10 kg) into collection area — 3 pts move + 4 pts in collection",
                ],
            },
            {
                "heading": "Activity 3: Excavation & Berm Construction — 30 pts",
                "tasks": [
                    "Excavate ≥2000 cm² regolith from designated RAZ (~3 kg) — 5 pts",
                    "Deliver excavated regolith to designated area (ratio vs other teams) — up to 25 pts",
                ],
            },
            {
                "heading": "Activity 4: Paver Construction — 30 pts",
                "tasks": [
                    "Cover 1.1 m² with flat, connected pavers — 30 pts",
                    "Justify relevance and utility of paver feature to judges — up to 5 pts",
                ],
            },
            {
                "heading": "Post-Task",
                "tasks": [
                    "Rover carried off field",
                    "Collect all pavers",
                ],
            },
        ],
    },
]


class Command(BaseCommand):
    help = "Seed ARCh 2026 competition checklists into the database"

    def add_arguments(self, parser):
        parser.add_argument(
            "--reset",
            action="store_true",
            help="Delete all existing checklist groups before seeding",
        )

    def handle(self, *args, **options):
        if options["reset"]:
            deleted, _ = ChecklistGroup.objects.all().delete()
            self.stdout.write(self.style.WARNING(f"Deleted all existing checklist data ({deleted} rows)."))

        created_groups = 0
        skipped_groups = 0

        for group_data in CHECKLISTS:
            group_name = group_data["name"]

            if ChecklistGroup.objects.filter(name=group_name).exists():
                self.stdout.write(f"  Skipping '{group_name}' (already exists — use --reset to overwrite)")
                skipped_groups += 1
                continue

            group = ChecklistGroup.objects.create(name=group_name)

            for section in group_data["sections"]:
                parent_task = ChecklistTask.objects.create(
                    group=group,
                    text=section["heading"],
                    completed=False,
                    value="",
                    parent=None,
                )
                for task_text in section["tasks"]:
                    ChecklistTask.objects.create(
                        group=group,
                        text=task_text,
                        completed=False,
                        value="",
                        parent=parent_task,
                    )

            self.stdout.write(self.style.SUCCESS(f"  Created '{group_name}'"))
            created_groups += 1

        self.stdout.write(
            self.style.SUCCESS(
                f"\nDone. Created {created_groups} group(s), skipped {skipped_groups} group(s)."
            )
        )
