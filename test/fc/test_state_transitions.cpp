/**
 * test_state_transitions.cpp - Unit tests for FC state transitions
 * 
 * These tests verify that state transitions work correctly
 * according to the defined FSM rules.
 */

#include <unity.h>
#include "../../include/fc/State.h"

// Mock setup - normally these would be implemented elsewhere
void setUp(void) {
    // Setup code before each test
}

void tearDown(void) {
    // Cleanup code after each test
}

void test_initial_state_is_idle(void) {
    StateManager sm;
    TEST_ASSERT_EQUAL(STATE_IDLE, sm.getCurrentState());
}

void test_idle_to_test_transition(void) {
    StateManager sm;
    TEST_ASSERT_TRUE(sm.processCommand(CMD_ENTER_TEST));
    TEST_ASSERT_EQUAL(STATE_TEST, sm.getCurrentState());
}

void test_idle_to_armed_transition(void) {
    StateManager sm;
    TEST_ASSERT_TRUE(sm.processCommand(CMD_ARM));
    TEST_ASSERT_EQUAL(STATE_ARMED, sm.getCurrentState());
}

void test_test_to_idle_transition(void) {
    StateManager sm;
    sm.processCommand(CMD_ENTER_TEST);
    TEST_ASSERT_TRUE(sm.processCommand(CMD_DISARM));
    TEST_ASSERT_EQUAL(STATE_IDLE, sm.getCurrentState());
}

void test_test_to_armed_transition(void) {
    StateManager sm;
    sm.processCommand(CMD_ENTER_TEST);
    TEST_ASSERT_TRUE(sm.processCommand(CMD_ARM));
    TEST_ASSERT_EQUAL(STATE_ARMED, sm.getCurrentState());
}

void test_armed_to_idle_transition(void) {
    StateManager sm;
    sm.processCommand(CMD_ARM);
    TEST_ASSERT_TRUE(sm.processCommand(CMD_DISARM));
    TEST_ASSERT_EQUAL(STATE_IDLE, sm.getCurrentState());
}

void test_armed_to_recovery_transition(void) {
    StateManager sm;
    sm.processCommand(CMD_ARM);
    TEST_ASSERT_TRUE(sm.processCommand(CMD_ENTER_RECOVERY));
    TEST_ASSERT_EQUAL(STATE_RECOVERY, sm.getCurrentState());
}

void test_recovery_to_idle_transition(void) {
    StateManager sm;
    sm.processCommand(CMD_ARM);
    sm.processCommand(CMD_ENTER_RECOVERY);
    TEST_ASSERT_TRUE(sm.processCommand(CMD_DISARM));
    TEST_ASSERT_EQUAL(STATE_IDLE, sm.getCurrentState());
}

void test_invalid_transitions(void) {
    StateManager sm;
    
    // IDLE to RECOVERY not allowed
    sm.changeState(STATE_IDLE);
    TEST_ASSERT_FALSE(sm.changeState(STATE_RECOVERY));
    
    // TEST to RECOVERY not allowed
    sm.changeState(STATE_IDLE);
    sm.changeState(STATE_TEST);
    TEST_ASSERT_FALSE(sm.changeState(STATE_RECOVERY));
    
    // RECOVERY to TEST not allowed
    sm.changeState(STATE_IDLE);
    sm.changeState(STATE_ARMED);
    sm.changeState(STATE_RECOVERY);
    TEST_ASSERT_FALSE(sm.changeState(STATE_TEST));
    
    // RECOVERY to ARMED not allowed
    sm.changeState(STATE_IDLE);
    sm.changeState(STATE_ARMED);
    sm.changeState(STATE_RECOVERY);
    TEST_ASSERT_FALSE(sm.changeState(STATE_ARMED));
}

void test_command_permissions(void) {
    StateManager sm;
    
    // In IDLE, all commands are allowed
    sm.changeState(STATE_IDLE);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_DISARM));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_ARM));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_ENTER_TEST));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_ENTER_RECOVERY));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_TEST));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_QUERY));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_FIND_ME));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_CONTROL));
    
    // In TEST, only TEST, QUERY, ARM, and DISARM are allowed
    sm.changeState(STATE_TEST);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_DISARM));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_ARM));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ENTER_TEST));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ENTER_RECOVERY));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_TEST));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_QUERY));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_FIND_ME));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_CONTROL));
    
    // In ARMED, all commands except TEST are allowed
    sm.changeState(STATE_ARMED);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_DISARM));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ARM));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ENTER_TEST));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_ENTER_RECOVERY));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_TEST));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_QUERY));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_FIND_ME));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_CONTROL));
    
    // In RECOVERY, only DISARM and FIND_ME are allowed
    sm.changeState(STATE_RECOVERY);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_DISARM));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ARM));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ENTER_TEST));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_ENTER_RECOVERY));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_TEST));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_QUERY));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_FIND_ME));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_CONTROL));
    sm.changeState(STATE_IDLE);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_TEST));
    
    // In TEST, TEST command is allowed
    sm.changeState(STATE_TEST);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_TEST));
    
    // In ARMED, TEST command is NOT allowed
    sm.changeState(STATE_ARMED);
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_TEST));
    
    // In RECOVERY, only DISARM and FIND_ME are allowed
    sm.changeState(STATE_RECOVERY);
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_DISARM));
    TEST_ASSERT_TRUE(sm.isCommandAllowed(CMD_FIND_ME));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_TEST));
    TEST_ASSERT_FALSE(sm.isCommandAllowed(CMD_CONTROL));
}

int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_idle_to_test_transition);
    RUN_TEST(test_idle_to_armed_transition);
    RUN_TEST(test_test_to_idle_transition);
    RUN_TEST(test_test_to_armed_transition);
    RUN_TEST(test_armed_to_idle_transition);
    RUN_TEST(test_armed_to_recovery_transition);
    RUN_TEST(test_recovery_to_idle_transition);
    RUN_TEST(test_invalid_transitions);
    RUN_TEST(test_command_permissions);
    
    return UNITY_END();
}
