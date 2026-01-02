# C++ Architectural Feature Workflow

A structured, human-in-the-loop workflow for implementing architectural C++ changes using Claude Code agents and markdown-based tickets.

## Overview

This workflow provides a 5-phase process for adding new architectural functionality to a C++ codebase:

1. **Design** — Create architecture design with PlantUML diagrams
2. **Design Review** — Validate design against quality criteria, identify prototype needs
3. **Prototype** — Validate uncertain aspects in isolation using Godbolt or standalone code
4. **Implementation** — Build the feature following the validated design
5. **Implementation Review** — Final verification before merge

**Key Features:**
- **Ticket-based** — Each feature has a markdown ticket that tracks requirements, status, and feedback
- **Human-in-the-loop** — You review and provide feedback between each phase
- **Persistent context** — All decisions and artifacts are version-controllable files

## When to Use This Workflow

✅ **Use for:**
- Adding new classes that interact with existing libraries
- Introducing new abstraction layers
- Creating new interfaces between components
- Adding new subsystems or modules

❌ **Don't use for:**
- Bug fixes
- Minor refactors
- Changes contained within a single existing class
- Non-architectural changes

## Directory Structure

```
your-project/
├── tickets/                      # Feature tickets (start here!)
│   ├── lru-cache.md
│   └── async-loader.md
├── .claude/
│   ├── agents/
│   │   ├── workflow-orchestrator.md       # Reads tickets, coordinates workflow
│   │   ├── cpp-architect.md           # Design phase agent
│   │   ├── design-reviewer.md    # Design review agent
│   │   ├── cpp-prototyper.md         # Prototyping agent
│   │   ├── cpp-implementer.md        # Implementation agent
│   │   └── implementation-reviewer.md      # Implementation review agent
│   ├── templates/
│   │   ├── ticket.md.template    # Template for new tickets
│   │   └── design.puml.template  # PlantUML template
│   └── workflows/
│       └── feature-workflow.md   # Workflow documentation
├── docs/
│   └── designs/                  # Design artifacts (created by workflow)
│       └── {feature-name}/
└── prototypes/                   # Prototype code (created by workflow)
    └── {feature-name}/
```

## Quick Start (Ticket-Based)

### 1. Create a Ticket

Copy the template and fill in your requirements:

```bash
cp .claude/templates/ticket.md.template tickets/my-feature.md
```

Edit `tickets/my-feature.md`:
- Fill in Summary, Motivation, Requirements
- Add any design preferences or constraints
- Mark status as `[x] Ready for Design`

### 2. Process the Ticket

```
Process ticket: my-feature
```

The orchestrator will:
- Read your ticket
- Execute the appropriate phase (Design)
- Update the ticket with results
- Tell you what to review and what's next

### 3. Review and Provide Feedback

- Review the generated artifacts (design doc, PlantUML)
- Add feedback to the ticket's "Human Feedback" section
- Answer any open questions

### 4. Continue Processing

```
Process ticket: my-feature
```

Repeat until the ticket reaches "Approved — Ready to Merge".

### 5. Check Status Anytime

```
Status: my-feature
```

## Example Workflow Session

**You**: Process ticket: lru-cache

**Claude**: 
```
Reading ticket: tickets/lru-cache.md
Current Status: Ready for Design

Executing Designer agent...

✓ Ticket Update: lru-cache

Previous Status: Ready for Design
New Status: Design Complete — Awaiting Review

Artifacts Created:
- docs/designs/lru-cache/design.md
- docs/designs/lru-cache/lru-cache.puml

Next Steps:
- Review the design document
- Answer open questions about eviction policy
- Add feedback to ticket if needed
- Run "Process ticket: lru-cache" to proceed
```

**You**: *Reviews design, adds feedback to ticket*

**You**: Process ticket: lru-cache

**Claude**: 
```
Reading ticket: tickets/lru-cache.md
Current Status: Design Complete — Awaiting Review
Human Feedback: Incorporating...

Executing Design Reviewer agent...
```

...and so on through the workflow.

## Ticket Structure

Each ticket contains:

| Section | Purpose |
|---------|---------|
| **Status** | Checkboxes tracking current phase |
| **Metadata** | Priority, complexity, target components |
| **Summary/Motivation** | What and why |
| **Requirements** | Functional and non-functional requirements |
| **Constraints** | Technical, business, or timeline constraints |
| **Acceptance Criteria** | Measurable success conditions |
| **Design Decisions** | Your preferences, things to avoid, open questions |
| **References** | Related code, docs, and tickets |
| **Workflow Log** | Auto-updated phase completion records |
| **Human Feedback** | Your comments for agents to incorporate |

## Providing Feedback

The ticket's "Human Feedback" section is how you communicate with agents:

```markdown
## Human Feedback

### Feedback on Design
Use `std::shared_mutex` instead of `std::mutex` — we expect 
high read concurrency with rare writes.

Also, let's use `std::optional<V>` for cache lookups rather 
than throwing on cache miss.

### Feedback on Prototypes
The LRU eviction prototype looks good. For P2 (thread safety),
please also test with 100 concurrent readers.
```

When you run `Process ticket: my-feature`, agents will:
1. Read all feedback before starting
2. Incorporate it into their work
3. Mark feedback as addressed in the Workflow Log

## Phase Details

### Design Phase
**Agent:** `.claude/agents/cpp-architect.md`  
**Input:** Ticket requirements  
**Output:** Design document + PlantUML diagram

The C++ Architect:
- Reads your ticket's requirements and constraints
- Analyzes existing codebase architecture
- Creates a PlantUML diagram showing new/modified components
- Specifies interfaces with C++ code sketches
- Identifies test impact
- Notes open questions for human decision

### Design Review Phase
**Agent:** `.claude/agents/design-reviewer.md`  
**Input:** Design artifacts + your feedback

Evaluates:
- Architectural fit with existing codebase
- C++ design quality (RAII, const-correctness, Rule of 0/3/5, etc.)
- Feasibility (compile-time, runtime, integration)
- Testability

Produces:
- Detailed checklist assessment
- Risk identification with likelihood/impact
- **Prototype guidance** with success criteria and time boxes

### Prototype Phase
**Agent:** `.claude/agents/cpp-prototyper.md`  
**Input:** Design + prototype guidance

Validates design assumptions through:
- **Godbolt snippets** — Algorithm/template validation
- **Standalone executables** — More complex prototypes
- **Isolated test harnesses** — Behavior validation

Key principle: Prototypes are **isolated from the main codebase** to ensure any issues during implementation are integration-related, not fundamental flaws.

Produces:
- Prototype code in `prototypes/{feature}/`
- Results document with measurements
- Implementation ticket with validated technical decisions

### Implementation Phase
**Agent:** `.claude/agents/cpp-implementer.md`  
**Input:** Design + prototype results + implementation ticket

Implements:
- Headers and source files
- Unit tests alongside code
- Integration tests
- Updates to affected existing tests

### Implementation Review Phase
**Agent:** `.claude/agents/implementation-reviewer.md`  
**Input:** All artifacts + implemented code

Verifies:
- Design conformance
- Prototype learning application
- Code quality (memory safety, thread safety, error handling)
- Test coverage

Final gate before merge.

## Orchestrator Commands

| Command | Description |
|---------|-------------|
| `Process ticket: {name}` | Execute the next phase for a ticket |
| `Status: {name}` | Show current status without executing |
| `List tickets` | Show all tickets and their status |
| `New ticket: {name}` | Create a new ticket from template |

## Customization

### Adapting for Your Project

1. **Coding standards** — Update agents with your project's conventions
2. **Test framework** — Replace Catch2/GoogleTest references with your framework
3. **Build system** — Adjust CMake templates for your build setup
4. **File organization** — Modify path conventions to match your structure

### Adding Custom Checks

Edit the agent files to add project-specific criteria:

```markdown
# In .claude/agents/design-reviewer.md

### Project-Specific Checks
- [ ] Uses our custom memory allocator for collections
- [ ] Follows our error code conventions
- [ ] Compatible with our plugin architecture
```

## Troubleshooting

### Design Review Keeps Failing
- Check if design has fundamental issues
- Review the "Required Revisions" section
- Consider if the feature scope is too large

### Prototype Invalidates Design
- This is the workflow working correctly!
- Review prototype findings
- Return to Design phase with new information
- Don't force an invalid design into implementation

### Implementation Review Loops
- After 3 CHANGES REQUESTED, escalate
- May indicate design issue, not implementation issue
- Consider returning to earlier phases

## Best Practices

1. **Keep features focused** — One architectural change per workflow run
2. **Answer questions promptly** — Don't skip open questions; they'll resurface as bugs
3. **Trust the prototypes** — If something fails in prototype, fix the design
4. **Review artifacts carefully** — Catching issues early saves time
5. **Preserve prototype code** — Useful for future reference and tests

## License

[Your license here]