package org.firstinspires.ftc.teamcode.utils.math;

import java.util.*;

public class Combinations {

    public static <T> List<List<T>> getCombinations(List<T> items, int k) {
        List<List<T>> result = new ArrayList<>();
        backtrack(items, k, 0, new ArrayList<>(), result);
        return result;
    }

    private static <T> void backtrack(
            List<T> items,
            int k,
            int start,
            List<T> current,
            List<List<T>> result
    ) {
        if (current.size() == k) {
            result.add(new ArrayList<>(current)); // copy
            return;
        }

        for (int i = start; i < items.size(); i++) {
            current.add(items.get(i));
            backtrack(items, k, i + 1, current, result);
            current.remove(current.size() - 1);
        }
    }
}

