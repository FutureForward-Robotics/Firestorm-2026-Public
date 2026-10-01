package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

public class HiveTest {


    @Test
    public void whenRedAllianceAndFar_ExpectsValidIds() {
        int[] ids = Hive.tagIds(Alliance.RED, Hive.Cell.FAR);



        assertArrayEquals(new int[]{30, 31, 32, 33}, ids);
    }

    @Test
    public void whenRedAllianceAndAudience_ExpectsValidIds() {
        int[] ids = Hive.tagIds(Alliance.RED, Hive.Cell.AUDIENCE);



        assertArrayEquals(new int[]{34, 35, 36, 37}, ids);
    }

    @Test
    public void whenBlueAllianceAndFar_ExpectsValidIds() {
        int[] ids = Hive.tagIds(Alliance.BLUE, Hive.Cell.FAR);



        assertArrayEquals(new int[]{42, 43, 44, 45}, ids);
    }

    @Test
    public void whenBlueAllianceAndAudience_ExpectsValidIds() {
        int[] ids = Hive.tagIds(Alliance.BLUE, Hive.Cell.AUDIENCE);



        assertArrayEquals(new int[]{38, 39, 40, 41}, ids);
    }


    @Test
    public void whenRedAllianceWithRedTag_ExpectCell() {
        Hive.Cell cell = Hive.cellOf(Alliance.RED, 30);

        assertEquals(Hive.Cell.FAR, cell);
    }

    @Test
    public void whenRedAllianceWithBlueTag_ExpectNull() {
        Hive.Cell cell = Hive.cellOf(Alliance.RED, 38);

        assertNull(cell);
    }

    @Test
    public void whenBlueAllianceWithRedTag_ExpectCell() {
        Hive.Cell cell = Hive.cellOf(Alliance.BLUE, 30);

        assertNull(cell);
    }

    @Test
    public void whenBlueAllianceWithBlueTag_ExpectNull() {
        Hive.Cell cell = Hive.cellOf(Alliance.BLUE, 38);

        assertEquals(Hive.Cell.AUDIENCE, cell);
    }




}
