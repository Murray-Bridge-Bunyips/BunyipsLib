package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.DeferredTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class DeferredTaskTest extends SchedulerTests {
  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void deferredFunctionsTest(boolean interrupted) {
    AtomicBoolean init = new AtomicBoolean();
    AtomicBoolean periodic = new AtomicBoolean();
    AtomicInteger loops = new AtomicInteger();
    AtomicBoolean intr = new AtomicBoolean();
    AtomicBoolean nat = new AtomicBoolean();
    Task innerTask = Task.task()
            .init(() -> init.set(true))
            .periodic(() -> periodic.set(true))
            .addIsFinished((f) -> {
              loops.incrementAndGet();
              return f;
            })
            .addOnInterrupt(() -> intr.set(true))
            .onFinish(() -> nat.set(true));
    DeferredTask task = new DeferredTask(() -> innerTask);
    task.ensureInit();
    assertNotNull(task.getTask());
    assertTrue(init.get());
    task.execute();
    assertTrue(periodic.get());
    assertFalse(task.isFinished());
    assertEquals(1, loops.get());
    if (interrupted) {
      task.finish();
      assertTrue(nat.get());
      assertTrue(intr.get());
      assertEquals(1, loops.get());
    } else {
      nat.set(true);
      task.execute(); // need to update cond
      assertTrue(nat.get());
      assertFalse(intr.get());
      assertEquals(2, loops.get());
    }
  }

//  @Test
//  void deferredSupplierOnlyCalledDuringInit() {
//    try (TaskScheduler scheduler = new TaskScheduler()) {
//      Supplier<Task> supplier = (Supplier<Task>) mock(Supplier.class);
//      when(supplier.get()).thenReturn(Tasks.none(), Tasks.none());
//
//      DeferredTask task = new DeferredTask(supplier, Set.of());
//      verify(supplier, never()).get();
//
//      scheduler.schedule(task);
//      verify(supplier, only()).get();
//      scheduler.run();
//
//      scheduler.schedule(task);
//      verify(supplier, times(2)).get();
//    }
//  }
//
//  @Test
//  void deferredRequirementsTest() {
//    Subsystem subsystem = new Subsystem() {};
//    DeferredTask task = new DeferredTask(Tasks::none, Set.of(subsystem));
//
//    assertTrue(task.getRequirements().contains(subsystem));
//  }
//
//  @Test
//  void deferredNullTaskTest() {
//    DeferredTask task = new DeferredTask(() -> null, Set.of());
//    assertDoesNotThrow(
//        () -> { task.execute()
//        });
//  }
}