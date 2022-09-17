import mill._, scalalib._

val spinalVersion = "1.6.4"

object opengdemu extends ScalaModule {
  Console.println("OpenGDEMU ScalaModule")
  def scalaVersion = "2.11.12"
  override def millSourcePath = os.pwd
  def ivyDeps = Agg(
    ivy"com.github.spinalhdl::spinalhdl-core:$spinalVersion",
    ivy"com.github.spinalhdl::spinalhdl-lib:$spinalVersion"
  )
  def scalacPluginIvyDeps = Agg(ivy"com.github.spinalhdl::spinalhdl-idsl-plugin:$spinalVersion")
}