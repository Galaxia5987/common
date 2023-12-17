package lib.logic;

import java.util.function.BooleanSupplier;

public class LogicSupplier implements BooleanSupplier {

    private BooleanSupplier supplier;

    public LogicSupplier(BooleanSupplier supplier) {
        this.supplier = supplier;
    }

    public LogicSupplier or(BooleanSupplier newSupplier) {
        supplier = () -> supplier.getAsBoolean() || newSupplier.getAsBoolean();
        return this;
    }

    public LogicSupplier and(BooleanSupplier newSupplier) {
        supplier = () -> supplier.getAsBoolean() && newSupplier.getAsBoolean();
        return this;
    }

    public LogicSupplier xor(BooleanSupplier newSupplier) {
        supplier = () -> supplier.getAsBoolean() ^ newSupplier.getAsBoolean();
        return this;
    }

    public LogicSupplier nor(BooleanSupplier newSupplier) {
        supplier = () -> !(supplier.getAsBoolean() || newSupplier.getAsBoolean());
        return this;
    }

    public LogicSupplier nand(BooleanSupplier newSupplier) {
        supplier = () -> !(supplier.getAsBoolean() && newSupplier.getAsBoolean());
        return this;
    }

    public LogicSupplier xnor(BooleanSupplier newSupplier) {
        supplier = () -> supplier.getAsBoolean() == newSupplier.getAsBoolean();
        return this;
    }

    public LogicSupplier not() {
        supplier = () -> !supplier.getAsBoolean();
        return this;
    }

    @Override
    public boolean getAsBoolean() {
        return supplier.getAsBoolean();
    }
}
