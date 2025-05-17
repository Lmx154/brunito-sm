# Brunito Project Instruction Prompts

## Phase Completion Prompt

```
You are assisting with the Brunito flight control system project. Your task is to implement Phase {X} of the project.

Please carefully analyze the following documents to understand the project structure and requirements:

1. First, read COMMUNICATION.md thoroughly as it contains the complete software architecture, data structures, message formats, and state machine design that must be followed precisely.
   
2. Review ROADMAP.md to understand the specific tasks and requirements for Phase {X} that you need to implement.
   
3. Consult HWSETUP.md for hardware specifications, pinouts, and component configurations that your implementation will target.

When implementing Phase {X}:
- Follow the architectural patterns in COMMUNICATION.md exactly
- Ensure your code meets all criteria specified in the "Done when" column of ROADMAP.md for Phase {X}
- Use the correct pins and hardware configurations from HWSETUP.md
- Write clean, well-commented code that follows the established file structure
- Do not create additional testing scripts - the user will test the implementation on actual hardware
- Include only the necessary code to implement the specific phase requirements

Please provide your implementation as code blocks organized by file, with clear explanations of your approach. Stop after completing the phase implementation so the user can test the changes on hardware.
```

## Issue Resolution Prompt

```
You are assisting with debugging an issue in the Brunito flight control system project, specifically in Phase {X}.

I've encountered the following error/issue during testing:

{INSERT LOG OR ISSUE DESCRIPTION HERE}

Please analyze this issue and suggest fixes that:
1. Address the specific problem shown in the logs/error
2. STRICTLY maintain compliance with the architecture and design in COMMUNICATION.md
3. Stay within the scope of Phase {X} as defined in ROADMAP.md
4. Use the correct hardware configurations from HWSETUP.md

The fix should be minimal and focused on resolving just this specific issue without introducing additional features or redesigning aspects that are working correctly. Your solution must remain consistent with the established message formats, state transitions, and component interactions defined in COMMUNICATION.md.

Please provide specific code changes with clear explanations of what was causing the issue and how your changes resolve it.
```

## Custom Instruction Prompt

```
You are assisting with the Brunito flight control system project. Your task is to implement a specific feature or functionality as described below:

{INSERT CUSTOM TASK DESCRIPTION HERE}

Please carefully analyze the following documents to understand the project structure and requirements:

1. First, read COMMUNICATION.md thoroughly as it contains the complete software architecture, data structures, message formats, and state machine design that must be followed precisely.
   
2. Consult HWSETUP.md for hardware specifications, pinouts, and component configurations that your implementation will target.

When implementing this functionality:
- Follow the architectural patterns in COMMUNICATION.md exactly
- Use the correct pins and hardware configurations from HWSETUP.md
- Write clean, well-commented code that follows the established file structure
- Do not create additional testing scripts - the user will test the implementation on actual hardware
- Include only the necessary code to implement the requested functionality

Please provide your implementation as code blocks organized by file, with clear explanations of your approach. Stop after completing the implementation so the user can test the changes on hardware.
```

Feel free to customize these prompts with specific phase numbers and details as needed.
