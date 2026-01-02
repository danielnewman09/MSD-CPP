#!/bin/bash
# =============================================================================
# Feature Workflow Runner
# Usage: ./run-workflow.sh <phase> <feature-name> [additional args]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 <phase> <feature-name> [options]"
    echo ""
    echo "Phases:"
    echo "  design              Run the Design phase"
    echo "  design-review       Run the Design Review phase"
    echo "  prototype           Run the Prototype phase"
    echo "  implement           Run the Implementation phase"
    echo "  impl-review         Run the Implementation Review phase"
    echo "  doc-update          Run the Documentation Update phase"
    echo "  status              Show current status of a feature"
    echo ""
    echo "Options:"
    echo "  --description, -d   Feature description (required for design phase)"
    echo "  --feedback, -f      Path to feedback file for revisions"
    echo "  --help, -h          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 design my-feature -d \"Add new caching layer for database queries\""
    echo "  $0 design-review my-feature"
    echo "  $0 prototype my-feature"
    echo "  $0 implement my-feature"
    echo "  $0 impl-review my-feature"
    echo "  $0 doc-update my-feature"
    echo "  $0 status my-feature"
}

check_feature_exists() {
    local feature_name=$1
    local design_dir="$PROJECT_ROOT/docs/designs/$feature_name"
    
    if [ ! -d "$design_dir" ]; then
        echo -e "${RED}Error: Feature '$feature_name' not found.${NC}"
        echo "Design directory does not exist: $design_dir"
        echo ""
        echo "To start a new feature, run:"
        echo "  $0 design $feature_name -d \"Your feature description\""
        exit 1
    fi
}

show_status() {
    local feature_name=$1
    local design_dir="$PROJECT_ROOT/docs/designs/$feature_name"
    
    echo -e "${BLUE}=== Feature Status: $feature_name ===${NC}"
    echo ""
    
    # Check each artifact
    local artifacts=(
        "design.md:Design"
        "${feature_name}.puml:PlantUML"
        "prototype-results.md:Prototype Results"
        "implementation-notes.md:Implementation Notes"
        "implementation-review.md:Implementation Review"
    )
    
    for artifact in "${artifacts[@]}"; do
        local file="${artifact%%:*}"
        local name="${artifact##*:}"
        local path="$design_dir/$file"
        
        if [ -f "$path" ]; then
            echo -e "  ${GREEN}✓${NC} $name: $path"
            
            # Check for review status in design.md
            if [ "$file" == "design.md" ]; then
                if grep -q "Status: APPROVED" "$path" 2>/dev/null; then
                    echo -e "    ${GREEN}└─ Design Review: APPROVED${NC}"
                elif grep -q "Status: NEEDS REVISION" "$path" 2>/dev/null; then
                    echo -e "    ${YELLOW}└─ Design Review: NEEDS REVISION${NC}"
                elif grep -q "Status: BLOCKED" "$path" 2>/dev/null; then
                    echo -e "    ${RED}└─ Design Review: BLOCKED${NC}"
                fi
            fi
            
            # Check for impl review status
            if [ "$file" == "implementation-review.md" ]; then
                if grep -q "Status.*APPROVED" "$path" 2>/dev/null; then
                    echo -e "    ${GREEN}└─ APPROVED - Ready to merge${NC}"
                elif grep -q "Status.*CHANGES REQUESTED" "$path" 2>/dev/null; then
                    echo -e "    ${YELLOW}└─ CHANGES REQUESTED${NC}"
                fi
            fi
        else
            echo -e "  ${YELLOW}○${NC} $name: Not yet created"
        fi
    done
    
    # Check prototypes
    local proto_dir="$PROJECT_ROOT/prototypes/$feature_name"
    if [ -d "$proto_dir" ]; then
        echo -e "  ${GREEN}✓${NC} Prototype code: $proto_dir"
        ls -la "$proto_dir" 2>/dev/null | grep "^d" | awk '{print "    └─ " $NF}'
    fi
    
    echo ""
}

run_phase() {
    local phase=$1
    local feature_name=$2
    local description=$3
    local feedback=$4
    
    local agent_file=""
    local input_context=""
    
    case $phase in
        design)
            agent_file="$PROJECT_ROOT/.claude/agents/cpp-architect.md"
            if [ -z "$description" ]; then
                echo -e "${RED}Error: Description required for design phase${NC}"
                echo "Use: $0 design $feature_name -d \"Description of the feature\""
                exit 1
            fi
            # Create feature directory
            mkdir -p "$PROJECT_ROOT/docs/designs/$feature_name"
            input_context="Feature '$feature_name': $description"
            ;;
        design-review)
            agent_file="$PROJECT_ROOT/.claude/agents/design-reviewer.md"
            check_feature_exists "$feature_name"
            input_context="Review design at docs/designs/$feature_name/design.md"
            ;;
        prototype)
            agent_file="$PROJECT_ROOT/.claude/agents/cpp-prototyper.md"
            check_feature_exists "$feature_name"
            input_context="Execute prototypes based on docs/designs/$feature_name/design.md"
            ;;
        implement)
            agent_file="$PROJECT_ROOT/.claude/agents/cpp-implementer.md"
            check_feature_exists "$feature_name"
            input_context="Implement feature based on docs/designs/$feature_name/"
            ;;
        impl-review)
            agent_file="$PROJECT_ROOT/.claude/agents/implementation-reviewer.md"
            check_feature_exists "$feature_name"
            input_context="Review implementation based on docs/designs/$feature_name/"
            ;;
        doc-update)
            agent_file="$PROJECT_ROOT/.claude/agents/docs-updater.md"
            check_feature_exists "$feature_name"
            input_context="Update the documentation for the code based on docs/designs/$feature_name/"
            ;;
        *)
            echo -e "${RED}Unknown phase: $phase${NC}"
            print_usage
            exit 1
            ;;
    esac
    
    # Add feedback if provided
    if [ -n "$feedback" ] && [ -f "$feedback" ]; then
        input_context="$input_context

Human Feedback:
$(cat "$feedback")"
    fi
    
    echo -e "${BLUE}=== Running Phase: $phase ===${NC}"
    echo -e "Feature: $feature_name"
    echo -e "Agent: $agent_file"
    echo ""
    
    # Output the claude command (actual execution depends on your setup)
    echo -e "${YELLOW}Execute with Claude:${NC}"
    echo "---"
    echo "Read the agent instructions at: $agent_file"
    echo ""
    echo "Input context:"
    echo "$input_context"
    echo "---"
    echo ""
    echo -e "${GREEN}After completion, review artifacts in: docs/designs/$feature_name/${NC}"
}

# =============================================================================
# Main
# =============================================================================

if [ $# -lt 1 ]; then
    print_usage
    exit 1
fi

PHASE=$1
FEATURE_NAME=$2
DESCRIPTION=""
FEEDBACK=""

# Parse additional arguments
shift 2 2>/dev/null || true
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--description)
            DESCRIPTION="$2"
            shift 2
            ;;
        -f|--feedback)
            FEEDBACK="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

case $PHASE in
    status)
        if [ -z "$FEATURE_NAME" ]; then
            echo "Usage: $0 status <feature-name>"
            exit 1
        fi
        check_feature_exists "$FEATURE_NAME"
        show_status "$FEATURE_NAME"
        ;;
    -h|--help|help)
        print_usage
        ;;
    *)
        if [ -z "$FEATURE_NAME" ]; then
            echo -e "${RED}Error: Feature name required${NC}"
            print_usage
            exit 1
        fi
        run_phase "$PHASE" "$FEATURE_NAME" "$DESCRIPTION" "$FEEDBACK"
        ;;
esac