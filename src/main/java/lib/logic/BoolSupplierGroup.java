package lib.logic;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class BoolSupplierGroup implements BooleanSupplier {

    private final ArrayList<BooleanSupplier> suppliers = new ArrayList<>();

    public BoolSupplierGroup(BooleanSupplier... suppliers) {
        this.suppliers.addAll(List.of(suppliers));
    }

    @Override
    public boolean getAsBoolean() {
        boolean result = false;
        for (BooleanSupplier supplier : suppliers) {
            result |= supplier.getAsBoolean();
        }
        return result;
    }
}
