from NHK2024_Raspi_Library import MainController

class R2Controller(MainController):
    def main(self):
        pass

if __name__ == "__main__":
    host_name = "R2.local"
    port = 12345
    controller = R2Controller(host_name=host_name, port=port)
    controller.main()